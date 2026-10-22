#include "rtthread.h"
#include "sensor_service.h"

#ifdef RT_USING_FINSH
#include "rthw.h"
#include "finsh.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* 数据集与CSV格式版本。修改字段或字段语义时必须递增版本。 */
#define SENSOR_DATASET_VERSION                 "task10_v1"
#define SENSOR_CAPTURE_FORMAT_VERSION          1U

/* 数据集读取统一使用20 ms轮询，不改变sensor服务的采样周期。 */
#define SENSOR_CAPTURE_PERIOD_MS               20U
#define SENSOR_CAPTURE_START_LEAD_MS           100U

/* 防止FinSH命令因参数错误或误操作无限运行。 */
#define SENSOR_CAPTURE_DURATION_MIN_S          1U
#define SENSOR_CAPTURE_DURATION_MAX_S          60U
#define SENSOR_CAPTURE_SEGMENT_ID_MAX_LENGTH   31U
/*
 * 当前RT_CONSOLEBUF_SIZE只有128字节，不能直接使用rt_kprintf
 * 输出完整CSV记录。局部缓冲区只占用FinSH线程栈，当前FinSH
 * 线程栈为4096字节，不增加长期静态RAM占用。
 */
#define SENSOR_DATASET_LINE_BUFFER_SIZE        512U

/**
 * @brief 一次有限数据采集的累计统计
 *
 * 这些字段只属于FinSH数据采集命令，不进入sensor服务内部状态。
 * 所有异常行仍会写入原始UART日志，统计只用于解释采集质量。
 */
typedef struct
{
    rt_uint32_t poll_count;              //实际执行快照读取的总次数
    rt_uint32_t new_sample_count;        //检测到IMU样本序号更新的次数
    rt_uint32_t duplicate_count;         //连续两次读取到相同IMU样本序号的次数
    rt_uint32_t seq_gap_count;           //IMU样本序号跳变期间累计缺失的样本数量
    rt_uint32_t invalid_count;           //快照成功但IMU状态或样本无效的次数
    rt_uint32_t stale_count;             //读取到IMU last-good样本已经过期的次数
    rt_uint32_t snapshot_error_count;    //调用sensor_service_get_snapshot失败的次数
    rt_uint32_t config_mismatch_count;   //样本配置版本与采集开始时冻结版本不一致的次数

    rt_uint32_t late_poll_count;         //实际读取时刻晚于计划读取时刻的次数
    rt_uint32_t schedule_skip_count;     //为避免追赶过期周期而主动跳过的计划读取次数
    rt_tick_t max_late_ticks;            //单次读取相对计划时刻的最大迟到tick数

    rt_tick_t first_tick;                //第一次实际执行快照读取时的系统tick
    rt_tick_t last_tick;                 //最后一次实际执行快照读取时的系统tick
} sensor_dataset_capture_stats_t;

/*
 * 防止两个FinSH上下文同时执行数据采集。
 * 该变量只能在短临界区内访问，volatile不能代替并发保护。
 */
static rt_bool_t g_imu_dataset_capture_active = RT_FALSE;

//私有函数声明
static rt_tick_t sensor_dataset_ms_to_tick(rt_uint32_t milliseconds);           //将毫秒转换为非零RT-Thread系统tick
static rt_bool_t sensor_dataset_tick_reached(rt_tick_t now,rt_tick_t deadline); //使用支持tick回绕的方式判断期限是否已经到达
static rt_bool_t sensor_dataset_try_claim(void);                                //在短临界区内申请数据采集命令的唯一执行权
static void sensor_dataset_release(void);                                       //在短临界区内释放数据采集命令的执行权
static rt_bool_t sensor_dataset_token_is_valid(const char *text,
                                                rt_size_t maximum_length);      //检查CSV文本参数的长度和字符是否合法
static rt_err_t sensor_dataset_parse_duration(const char *text,
                                             rt_uint32_t *duration_s);          //解析并检查数据采集持续时间参数
static rt_bool_t sensor_dataset_split_is_valid(const char *text);               //检查数据集划分是否为design或validation

static rt_bool_t sensor_dataset_class_is_valid(const char *text);               //检查场景类别是否为still、walk或nonstep
static rt_err_t sensor_dataset_outputf(const char *format, ...);                //在局部大缓冲区中格式化并一次性输出完整记录
static rt_err_t sensor_dataset_print_data(rt_uint32_t capture_index,
                                            rt_tick_t poll_tick,
                                            rt_tick_t late_ticks,
                                            rt_err_t snapshot_result,
                                            rt_bool_t is_new_sample,
                                            rt_tick_t delta_tick,
                                            const sensor_snapshot_t *snapshot); //按固定CSV字段顺序输出一行快照数据

static int imu_dataset_capture(int argc,char **argv);                           //执行有限时长的IMU数据集快照采集并输出统计结果

/*
 * 简介：将毫秒时间转换为RT-Thread系统tick。
 * 输入：milliseconds为需要转换的毫秒数。
 * 返回值：对应的系统tick数；
 *         输入为0时返回0；
 *         非零毫秒转换结果至少返回1 tick。
 * 说明：不同系统tick频率可能使较短的毫秒值转换为0；
 *       对非零输入强制返回至少1 tick，可以防止采集调度
 *       因等待时间为0而形成忙循环。
 */
static rt_tick_t sensor_dataset_ms_to_tick(rt_uint32_t milliseconds)
{
    rt_tick_t ticks;

    ticks = rt_tick_from_millisecond((rt_int32_t)milliseconds);

    if ((milliseconds != 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }

    return ticks;
}

/*
 * 简介：判断当前系统tick是否已经到达或超过绝对调度期限。
 * 输入：now为当前系统tick；
 *       deadline为需要判断的绝对期限tick。
 * 返回值：已经到达或超过期限返回RT_TRUE；
 *         期限仍在未来返回RT_FALSE。
 * 说明：通过将now与deadline的无符号差值转换为有符号数，
 *       可以在比较区间小于半个tick计数范围时正确处理一次回绕。
 *       本命令最长运行60秒，满足该比较方法的使用条件。
 */
static rt_bool_t sensor_dataset_tick_reached(rt_tick_t now,rt_tick_t deadline)
{
    return (((rt_int32_t)(now - deadline)) >= 0)
        ? RT_TRUE
        : RT_FALSE;
}

/*
 * 简介：申请IMU数据采集命令的唯一执行权。
 * 输入：无。
 * 返回值：成功取得执行权返回RT_TRUE；
 *         已有采集命令正在运行返回RT_FALSE。
 * 说明：禁止两个采集命令同时输出，避免CSV记录交错而破坏
 *       原始证据。临界区内只检查和修改一个布尔状态，
 *       不执行日志、延时、快照读取或其他阻塞操作。
 */
static rt_bool_t sensor_dataset_try_claim(void)
{
    rt_base_t level;
    rt_bool_t claimed = RT_FALSE;

    level = rt_hw_interrupt_disable();

    if (g_imu_dataset_capture_active == RT_FALSE)
    {
        g_imu_dataset_capture_active = RT_TRUE;
        claimed = RT_TRUE;
    }

    rt_hw_interrupt_enable(level);

    return claimed;
}

/*
 * 简介：释放IMU数据采集命令的唯一执行权。
 * 输入：无。
 * 返回值：无。
 * 说明：采集正常完成或中途失败时都必须调用本函数，
 *       否则后续采集命令会一直被误判为正在运行。
 *       临界区内只修改一个布尔状态，不执行日志、延时、
 *       快照读取或其他阻塞操作。
 */
static void sensor_dataset_release(void)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    g_imu_dataset_capture_active = RT_FALSE;
    rt_hw_interrupt_enable(level);
}

/*
 * 简介：检查写入CSV元数据的字符串参数是否合法。
 * 输入：text为需要检查的以空字符结尾的字符串；
 *       maximum_length为允许的最大字符数，不包含结尾空字符。
 * 返回值：字符串非空、长度未超过限制且所有字符均合法时
 *         返回RT_TRUE；否则返回RT_FALSE。
 * 说明：segment_id等参数会原样写入CSV，因此只允许字母、
 *       数字、下划线和短横线。这样可以防止逗号、空格、
 *       换行及其他控制字符破坏CSV字段和证据解析过程。
 */
static rt_bool_t sensor_dataset_token_is_valid(const char *text,rt_size_t maximum_length)
{
    rt_size_t index;
    char character;

    if ((text == RT_NULL) ||
        (text[0] == '\0') ||
        (maximum_length == 0U))
    {
        return RT_FALSE;
    }

    for (index = 0U; text[index] != '\0'; index++)
    {
        if (index >= maximum_length)
        {
            return RT_FALSE;
        }

        character = text[index];

        if (((character >= 'A') && (character <= 'Z')) ||
            ((character >= 'a') && (character <= 'z')) ||
            ((character >= '0') && (character <= '9')) ||
            (character == '_') ||
            (character == '-'))
        {
            continue;
        }

        return RT_FALSE;
    }

    return RT_TRUE;
}

/*
 * 简介：解析并检查有限数据采集时长参数。
 * 输入：text为FinSH命令传入的十进制秒数字符串；
 *       duration_s为解析后的采集时长输出地址。
 * 返回值：参数完整、未溢出且位于允许范围时返回RT_EOK；
 *         空指针、格式错误、部分数字、溢出或超出范围时
 *         返回-RT_EINVAL。
 * 说明：同时检查strtoul的errno、结束指针和数值范围，
 *       防止负数、混合字符及溢出值被误认为有效时长。
 *       最大时长限制为60秒，保证命令执行时间始终有界。
 */
static rt_err_t sensor_dataset_parse_duration(const char *text, rt_uint32_t *duration_s)
{
    char *end_pointer;
    unsigned long value;

    if ((text == RT_NULL) || (duration_s == RT_NULL))
    {
        return -RT_EINVAL;
    }

    errno = 0;
    end_pointer = RT_NULL;
    value = strtoul(text, &end_pointer, 10);

    if ((errno != 0) ||
        (end_pointer == text) ||
        (*end_pointer != '\0') ||
        (value < SENSOR_CAPTURE_DURATION_MIN_S) ||
        (value > SENSOR_CAPTURE_DURATION_MAX_S))
    {
        return -RT_EINVAL;
    }

    *duration_s = (rt_uint32_t)value;

    return RT_EOK;
}

/*
 * 简介：检查数据集划分名称是否符合冻结协议。
 * 输入：text为需要检查的数据集划分字符串。
 * 返回值：字符串为design或validation时返回RT_TRUE；
 *         空指针或其他字符串返回RT_FALSE。
 * 说明：只接受协议冻结的两种划分名称，防止采集结果出现
 *       无法归类的split，并保证CSV、manifest和文件名语义一致。
 */
static rt_bool_t sensor_dataset_split_is_valid(const char *text)
{
    if (text == RT_NULL)
    {
        return RT_FALSE;
    }

    return ((strcmp(text, "design") == 0) ||
            (strcmp(text, "validation") == 0))
        ? RT_TRUE
        : RT_FALSE;
}

/*
 * 简介：检查动作类别名称是否符合冻结协议。
 * 输入：text为需要检查的动作类别字符串。
 * 返回值：字符串为still、walk或nonstep时返回RT_TRUE；
 *         空指针或其他字符串返回RT_FALSE。
 * 说明：只接受协议冻结的三种动作类别，防止采集片段出现
 *       未定义分类，并保证CSV、manifest和文件名语义一致。
 */
static rt_bool_t sensor_dataset_class_is_valid(const char *text)
{
    if (text == RT_NULL)
    {
        return RT_FALSE;
    }

    return ((strcmp(text, "still") == 0) ||
            (strcmp(text, "walk") == 0) ||
            (strcmp(text, "nonstep") == 0))
        ? RT_TRUE
        : RT_FALSE;
}

/*
 * 简介：绕开rt_kprintf的128字节全局格式化缓冲区，输出一条完整记录。
 * 输入：format及其可变参数组成一条以换行结尾的文本记录。
 * 返回值：完整格式化并提交输出返回RT_EOK；
 *         参数无效返回-RT_EINVAL；
 *         结果超过本地缓冲区返回-RT_EFULL。
 * 说明：
 * 1. rt_vsnprintf使用本函数的512字节局部缓冲区；
 * 2. rt_kputs不经过RT_CONSOLEBUF_SIZE，而是按实际字符串长度写控制台；
 * 3. 一条记录只调用一次rt_kputs，降低其他线程日志插入行中间的风险；
 * 4. 本函数运行在FinSH线程，不在sensor快照锁内调用。
 */
static rt_err_t sensor_dataset_outputf(const char *format, ...)
{
    char line_buffer[SENSOR_DATASET_LINE_BUFFER_SIZE];
    va_list arguments;
    rt_int32_t length;

    if (format == RT_NULL)
    {
        return -RT_EINVAL;
    }

    va_start(arguments, format);

    length = rt_vsnprintf(
        line_buffer,
        sizeof(line_buffer),
        format,
        arguments);

    va_end(arguments);

    /*
     * rt_vsnprintf返回完整结果原本需要的字符数。
     * 等于或超过缓冲区大小说明记录被截断，禁止输出半条CSV。
     */
    if ((length < 0) ||
        ((rt_size_t)length >= sizeof(line_buffer)))
    {
        return -RT_EFULL;
    }

    rt_kputs(line_buffer);

    return RT_EOK;
}



/*
 * 简介：按照冻结字段顺序输出一行IMU数据集CSV记录。
 * 输入：capture_index为当前实际输出的数据行编号；
 *       poll_tick为调用快照复制接口前取得的系统tick；
 *       late_ticks为本次读取相对绝对调度期限迟到的tick数；
 *       snapshot_result为sensor_service_get_snapshot()返回值；
 *       is_new_sample表示本行是否包含新的IMU样本序号；
 *       delta_tick为相邻两个新IMU样本sample_tick的差值；
 *       snapshot为成功复制到调用者缓冲区的一致快照。
 * 返回值：完整输出返回RT_EOK；
 *         格式化结果超过本地缓冲区返回-RT_EFULL。
 * 说明：快照读取失败时仍输出一行D记录，并通过snapshot_result、
 *       state=-1、valid=0和其余默认值保存失败位置，不能静默删除
 *       原始数据中的异常。函数只读取调用者持有的快照副本，
 *       不持有sensor服务锁，也不访问I2C或修改传感器状态。
 */
static rt_err_t sensor_dataset_print_data(rt_uint32_t capture_index,
                                        rt_tick_t poll_tick,
                                        rt_tick_t late_ticks,
                                        rt_err_t snapshot_result,
                                        rt_bool_t is_new_sample,
                                        rt_tick_t delta_tick,
                                        const sensor_snapshot_t *snapshot)
{
    /*
     * 默认构造一行“快照读取失败”记录。
     * 如果快照读取成功，后续代码再用实际字段整体替换这些默认值。
     */
    rt_uint32_t publish_seq = 0U;
    rt_uint32_t imu_seq = 0U;
    rt_tick_t sample_tick = 0U;
    rt_int32_t state = -1;
    rt_uint32_t valid = 0U;
    rt_uint32_t stale = 1U;
    rt_tick_t age_ticks = 0U;
    rt_int32_t last_error = (rt_int32_t)snapshot_result;
    rt_uint32_t config_version = 0U;

    /* IMU六轴原始寄存器值，单位为LSB。 */
    rt_int32_t ax_raw = 0;
    rt_int32_t ay_raw = 0;
    rt_int32_t az_raw = 0;
    rt_int32_t gx_raw = 0;
    rt_int32_t gy_raw = 0;
    rt_int32_t gz_raw = 0;

    /* IMU六轴换算值，加速度单位为µg，角速度单位为µdps。 */
    rt_int32_t ax_ug = 0;
    rt_int32_t ay_ug = 0;
    rt_int32_t az_ug = 0;
    rt_int32_t gx_udps = 0;
    rt_int32_t gy_udps = 0;
    rt_int32_t gz_udps = 0;

    /*
     * 只有快照接口返回成功且快照地址有效时，才读取快照字段。
     * 失败路径继续使用前面的默认值输出证据行。
     */
    if ((snapshot_result == RT_EOK) && (snapshot != RT_NULL))
    {
        /* 保存服务级发布序号和IMU独立样本序号。 */
        publish_seq = snapshot->publish_seq;
        imu_seq = snapshot->imu_status.seq;

        /*
         * sample_tick来自IMU完整12字节burst结束时刻，
         * 不能用本次poll_tick替代。
         */
        sample_tick = snapshot->imu.sample_tick;
        state = (rt_int32_t)snapshot->imu_status.state;

        /*
         * 服务状态和驱动样本必须同时有效，本行才能标记为有效。
         * 只检查其中一个会把未完成或不一致的数据误写成有效样本。
         */
        valid = ((snapshot->imu_status.valid == RT_TRUE) &&
                 (snapshot->imu.valid == RT_TRUE))
            ? 1U
            : 0U;

        /*
         * stale和age_ticks描述last-good样本的新鲜程度。
         * TEMP_FAILED或OFFLINE期间仍可能保留旧值，因此不能只看valid。
         */
        stale = (snapshot->imu_status.stale == RT_TRUE) ? 1U : 0U;
        age_ticks = snapshot->imu_status.age_ticks;
        last_error = (rt_int32_t)snapshot->imu_status.last_error;
        config_version = snapshot->imu.config_version;

        /* 复制三轴加速度和三轴角速度原始值。 */
        ax_raw = (rt_int32_t)snapshot->imu.accel_raw.x;
        ay_raw = (rt_int32_t)snapshot->imu.accel_raw.y;
        az_raw = (rt_int32_t)snapshot->imu.accel_raw.z;

        gx_raw = (rt_int32_t)snapshot->imu.gyro_raw.x;
        gy_raw = (rt_int32_t)snapshot->imu.gyro_raw.y;
        gz_raw = (rt_int32_t)snapshot->imu.gyro_raw.z;

        /* 复制经过冻结灵敏度换算的三轴加速度。 */
        ax_ug = snapshot->imu.accel_ug.x;
        ay_ug = snapshot->imu.accel_ug.y;
        az_ug = snapshot->imu.accel_ug.z;

        /* 复制经过冻结灵敏度换算的三轴角速度。 */
        gx_udps = snapshot->imu.gyro_udps.x;
        gy_udps = snapshot->imu.gyro_udps.y;
        gz_udps = snapshot->imu.gyro_udps.z;
    }

    /*
     * 使用一次rt_kprintf输出完整数据行，减少多个打印调用之间
     * 被其他线程日志插入的机会。这里输出的字段顺序必须与
     * #FIELDS记录完全一致，后续修改字段时必须递增格式版本。
     */
    return sensor_dataset_outputf(
        "D,%lu,%lu,%lu,%ld,%lu,%lu,%lu,%lu,%lu,"
        "%ld,%lu,%lu,%lu,%ld,%lu,"
        "%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%ld,%ld\n",
        (unsigned long)capture_index,
        (unsigned long)poll_tick,
        (unsigned long)late_ticks,
        (long)snapshot_result,
        (unsigned long)publish_seq,
        (unsigned long)imu_seq,
        (unsigned long)sample_tick,
        (unsigned long)delta_tick,
        (unsigned long)((is_new_sample == RT_TRUE) ? 1U : 0U),
        (long)state,
        (unsigned long)valid,
        (unsigned long)stale,
        (unsigned long)age_ticks,
        (long)last_error,
        (unsigned long)config_version,
        (long)ax_raw,
        (long)ay_raw,
        (long)az_raw,
        (long)gx_raw,
        (long)gy_raw,
        (long)gz_raw,
        (long)ax_ug,
        (long)ay_ug,
        (long)az_ug,
        (long)gx_udps,
        (long)gy_udps,
        (long)gz_udps);
}

/*
 * 简介：从统一sensor服务采集有限时长的IMU CSV数据。
 * 输入：argc和argv来自FinSH命令行；
 *       命令格式为：
 *       imu_dataset_capture <segment_id> <split> <class> <duration_s>
 * 返回值：采集完成且没有快照、有效性、过期或配置错误时返回RT_EOK；
 *         参数错误返回-RT_EINVAL；
 *         服务未运行返回-RT_ERROR；
 *         已有采集正在运行返回-RT_EBUSY；
 *         初始状态不满足采集条件返回-RT_EBUSY或-RT_EIO；
 *         采集中发现数据错误时，在输出完整#END后返回-RT_EIO。
 * 说明：
 * 1. 只调用sensor_service_get_snapshot()，不访问I2C或驱动寄存器；
 * 2. 使用绝对20 ms期限，逾期时跳过完整过期周期，不无限补采；
 * 3. 快照复制完成并释放服务锁后才分析和输出UART；
 * 4. 重复、无效、stale、序号跳变和快照失败均保留在原始记录中；
 * 5. 最长运行60秒，并通过唯一执行权禁止并发采集；
 * 6. 本函数运行在FinSH线程，不创建新的采集线程。
 */
static int imu_dataset_capture(int argc, char **argv)
{
    /* 采集启动前的状态检查快照，以及循环内使用的快照副本。 */
    sensor_snapshot_t initial_snapshot;
    sensor_snapshot_t snapshot;

    /* 本次采集累计统计，最终通过#END记录输出。 */
    sensor_dataset_capture_stats_t stats;

    /* 直接引用argv中的只读命令参数，不保存到全局状态。 */
    const char *segment_id;
    const char *split;
    const char *data_class;

    rt_uint32_t duration_s;
    rt_uint32_t capture_index = 0U;
    rt_uint32_t frozen_config_version;
    rt_uint32_t sequence_delta;
    rt_uint32_t skipped_periods;

    /*
     * 所有时间统一使用RT-Thread系统tick。
     * capture_start_tick是本段数据的时间零点；
     * next_poll_tick始终保存下一次绝对采集期限。
     */
    rt_tick_t period_ticks;
    rt_tick_t lead_ticks;
    rt_tick_t duration_ticks;
    rt_tick_t capture_start_tick;
    rt_tick_t capture_end_deadline;
    rt_tick_t capture_end_tick;
    rt_tick_t next_poll_tick;
    rt_tick_t now;
    rt_tick_t poll_tick;
    rt_tick_t late_ticks;
    rt_tick_t delta_tick = 0U;
    rt_tick_t previous_sample_tick = 0U;

    /*
     * previous_imu_seq和previous_sample_tick只描述上一份已经确认的
     * 新有效样本，用于识别重复、序号跳变和样本间隔。
     */
    rt_uint32_t previous_imu_seq = 0U;
    rt_bool_t previous_sample_valid = RT_FALSE;
    rt_bool_t current_sample_valid;
    rt_bool_t is_new_sample;

    rt_err_t result;
    rt_err_t command_result = RT_EOK;

    /*
     * 参数数量必须严格匹配。
     * 不接受缺失参数，也不静默忽略额外参数。
     */
    if (argc != 5)
    {
        rt_kprintf(
            "usage: imu_dataset_capture "
            "<segment_id> <design|validation> "
            "<still|walk|nonstep> <duration_s>\n");
        return -RT_EINVAL;
    }

    segment_id = argv[1];
    split = argv[2];
    data_class = argv[3];

    /*
     * segment_id会直接进入#META，因此必须先检查长度和字符集合，
     * 防止逗号、空格或控制字符破坏CSV结构。
     */
    if (sensor_dataset_token_is_valid(segment_id,SENSOR_CAPTURE_SEGMENT_ID_MAX_LENGTH) == RT_FALSE)
    {
        rt_kprintf(
            "[DATASET][ERR] invalid segment_id; "
            "use 1..31 letters digits '_' or '-'\n");
        return -RT_EINVAL;
    }

    /* split只能使用协议冻结的design或validation。 */
    if (sensor_dataset_split_is_valid(split) == RT_FALSE)
    {
        rt_kprintf(
            "[DATASET][ERR] invalid split=%s; "
            "expected design or validation\n",
            split);
        return -RT_EINVAL;
    }

    /* class只能使用协议冻结的三种动作类别。 */
    if (sensor_dataset_class_is_valid(data_class) == RT_FALSE)
    {
        rt_kprintf(
            "[DATASET][ERR] invalid class=%s; "
            "expected still walk or nonstep\n",
            data_class);
        return -RT_EINVAL;
    }

    /* 解析1～60秒的有限采集时长。 */
    result = sensor_dataset_parse_duration(argv[4], &duration_s);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DATASET][ERR] invalid duration_s=%s range=%u..%u\n",
            argv[4],
            SENSOR_CAPTURE_DURATION_MIN_S,
            SENSOR_CAPTURE_DURATION_MAX_S);
        return result;
    }

    /*
     * 数据集命令只能消费统一服务已经发布的快照。
     * 不允许为了采集数据而自行初始化驱动或直接访问IMU。
     */
    if (sensor_service_is_running() == RT_FALSE)
    {
        rt_kprintf(
            "[DATASET][ERR] sensor service is not running\n");
        return -RT_ERROR;
    }

    /*
     * 取得唯一执行权，防止两个FinSH采集命令同时输出，
     * 造成#META、D和#END记录互相交错。
     */
    if (sensor_dataset_try_claim() == RT_FALSE)
    {
        rt_kprintf("[DATASET][ERR] another capture is already active\n");
        return -RT_EBUSY;
    }

    /*
     * 所有结构先清零，使错误路径、尚未写入字段和最终统计
     * 都具有确定的初始值。
     */
    memset(&initial_snapshot, 0, sizeof(initial_snapshot));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&stats, 0, sizeof(stats));

    /*
     * 正式采集前先取得一份一致快照。
     * 服务线程已经启动不代表IMU当前已有新鲜有效样本。
     */
    result = sensor_service_get_snapshot(&initial_snapshot);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DATASET][ERR] initial snapshot failed result=%d\n",
            (int)result);

        /*
         * 取得执行权后的所有提前退出路径都必须释放执行权，
         * 否则后续命令会一直返回already active。
         */
        sensor_dataset_release();
        return result;
    }

    /*
     * 正式数据集要求IMU处于READY，同时服务状态、驱动样本有效，
     * 并且last-good样本尚未过期。
     */
    if ((initial_snapshot.imu_status.state !=
            SENSOR_DEVICE_STATE_READY) ||
        (initial_snapshot.imu_status.valid == RT_FALSE) ||
        (initial_snapshot.imu_status.stale == RT_TRUE) ||
        (initial_snapshot.imu.valid == RT_FALSE))
    {
        rt_kprintf(
            "[DATASET][ERR] IMU not ready "
            "state=%d valid=%u stale=%u sample_valid=%u error=%d\n",
            (int)initial_snapshot.imu_status.state,
            (unsigned int)initial_snapshot.imu_status.valid,
            (unsigned int)initial_snapshot.imu_status.stale,
            (unsigned int)initial_snapshot.imu.valid,
            (int)initial_snapshot.imu_status.last_error);

        sensor_dataset_release();
        return -RT_EBUSY;
    }

    /*
     * 样本和服务状态必须报告相同且非零的配置版本。
     * 采集开始后以该版本为冻结值，禁止同一片段混入不同配置。
     */
    if ((initial_snapshot.imu.config_version == 0U) ||
        (initial_snapshot.imu.config_version !=
            initial_snapshot.imu_status.config_version))
    {
        rt_kprintf(
            "[DATASET][ERR] IMU config mismatch "
            "sample=%lu status=%lu\n",
            (unsigned long)initial_snapshot.imu.config_version,
            (unsigned long)
                initial_snapshot.imu_status.config_version);

        sensor_dataset_release();
        return -RT_EIO;
    }

    frozen_config_version = initial_snapshot.imu.config_version;

    /*
     * 统一换算采样周期、表头输出预留时间和总采集时长。
     * period_ticks至少为1，避免零tick忙循环。
     */
    period_ticks =sensor_dataset_ms_to_tick(SENSOR_CAPTURE_PERIOD_MS);
    lead_ticks =sensor_dataset_ms_to_tick(SENSOR_CAPTURE_START_LEAD_MS);
    duration_ticks =(rt_tick_t)(duration_s * (rt_uint32_t)RT_TICK_PER_SECOND);

    /*
     * 预留100 ms输出#META和#FIELDS，避免第一行D记录因为
     * UART表头输出立即迟到。capture_start_tick仍是数据时间零点。
     */
    capture_start_tick = rt_tick_get() + lead_ticks;
    capture_end_deadline = capture_start_tick + duration_ticks;
    next_poll_tick = capture_start_tick;

    /*
     * 输出本段数据的固定元数据。
     * 同一dataset_version的正式片段必须保持这些采集语义一致。
     */
    result = sensor_dataset_outputf(
        "#META,dataset_version=%s,capture_format_version=%u,"
        "segment_id=%s,split=%s,class=%s,"
        "requested_duration_s=%lu,tick_hz=%u,"
        "nominal_period_ms=%u,period_ticks=%lu,"
        "imu_config_version=%lu,capture_start_tick=%lu\n",
        SENSOR_DATASET_VERSION,
        SENSOR_CAPTURE_FORMAT_VERSION,
        segment_id,
        split,
        data_class,
        (unsigned long)duration_s,
        (unsigned int)RT_TICK_PER_SECOND,
        SENSOR_CAPTURE_PERIOD_MS,
        (unsigned long)period_ticks,
        (unsigned long)frozen_config_version,
        (unsigned long)capture_start_tick);

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DATASET][ERR] META output failed result=%d\n",
            (int)result);

        sensor_dataset_release();
        return result;
    }

    /*
     * #FIELDS顺序必须与sensor_dataset_print_data()完全一致。
     * 增删字段或改变字段语义时必须递增capture_format_version。
     */
    rt_kputs(
        "#FIELDS,capture_index,poll_tick,late_ticks,"
        "snapshot_result,publish_seq,imu_seq,sample_tick,"
        "delta_tick,is_new_sample,state,valid,stale,age_ticks,"
        "last_error,config_version,"
        "ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,"
        "ax_ug,ay_ug,az_ug,gx_udps,gy_udps,gz_udps\n");

    /*
     * 表头输出完成后阻塞到正式采集起点。
     * 当前线程让出CPU，不使用忙等待。
     */
    now = rt_tick_get();
    if (sensor_dataset_tick_reached(now,capture_start_tick) == RT_FALSE)
    {
        rt_thread_delay(capture_start_tick - now);
    }

    /*
     * next_poll_tick始终表示绝对期限。
     * 如果UART输出或系统调度已经跨过完整周期，只保留最新期限，
     * 不连续补打全部过期数据行。
     */
    while ((rt_tick_t)(next_poll_tick - capture_start_tick) <duration_ticks)
    {
        now = rt_tick_get();

        /*
         * 如果当前时刻已经超过一个或多个完整采样周期，
         * 将绝对期限前移到最近周期，并统计被合并的计划周期。
         */
        if (sensor_dataset_tick_reached(now,next_poll_tick) == RT_TRUE)
        {
            late_ticks = (rt_tick_t)(now - next_poll_tick);

            if (late_ticks >= period_ticks)
            {
                skipped_periods =(rt_uint32_t)(late_ticks / period_ticks);

                next_poll_tick +=(rt_tick_t)(skipped_periods * period_ticks);

                stats.schedule_skip_count += skipped_periods;
            }
        }

        /*
         * 合并过期周期后可能已经到达整个采集区间末尾。
         * 此时直接结束循环，不能在正式时长之外继续补采。
         */
        if ((rt_tick_t)(next_poll_tick - capture_start_tick) >=duration_ticks)
        {
            break;
        }

        /*
         * 当前绝对期限仍在未来时阻塞等待。
         * rt_thread_delay允许其他线程继续运行，不形成忙循环。
         */
        now = rt_tick_get();

        if (sensor_dataset_tick_reached(now,next_poll_tick) == RT_FALSE)
        {
            rt_thread_delay(next_poll_tick - now);
        }

        /*
         * poll_tick固定在调用get_snapshot()之前取得，
         * 表示采集线程开始本次快照读取的时刻。
         */
        poll_tick = rt_tick_get();
        late_ticks = 0U;

        /*
         * 记录本次实际读取相对绝对期限的迟到量。
         * 迟到只描述采集线程调度，不修改IMU的sample_tick。
         */
        if (sensor_dataset_tick_reached(
                poll_tick,
                next_poll_tick) == RT_TRUE)
        {
            late_ticks =(rt_tick_t)(poll_tick - next_poll_tick);
        }

        if (late_ticks != 0U)
        {
            stats.late_poll_count++;

            if (late_ticks > stats.max_late_ticks)
            {
                stats.max_late_ticks = late_ticks;
            }
        }

        /* 保存第一行和最近一行数据的实际读取tick。 */
        if (stats.poll_count == 0U)
        {
            stats.first_tick = poll_tick;
        }

        stats.last_tick = poll_tick;
        stats.poll_count++;

        /*
         * 每轮先清空调用者快照，避免快照接口失败时误用上一轮数据。
         */
        memset(&snapshot, 0, sizeof(snapshot));

        /*
         * get_snapshot()在内部短锁内完成完整结构复制。
         * 返回后服务锁已经释放，后续分析和UART打印不会扩大锁区。
         */
        result = sensor_service_get_snapshot(&snapshot);

        /* 每一轮重新计算新样本标志和样本间隔。 */
        is_new_sample = RT_FALSE;
        delta_tick = 0U;

        if (result != RT_EOK)
        {
            /*
             * 快照失败仍会在后面输出一行D记录，
             * 不能从原始数据中静默删除失败位置。
             */
            stats.snapshot_error_count++;
        }
        else
        {
            /*
             * 服务状态和IMU驱动样本必须同时有效，
             * 才能把当前last-good数据当作有效样本。
             */
            current_sample_valid =
                ((snapshot.imu_status.valid == RT_TRUE) &&
                 (snapshot.imu.valid == RT_TRUE))
                ? RT_TRUE
                : RT_FALSE;

            if (current_sample_valid == RT_FALSE)
            {
                stats.invalid_count++;
            }

            /*
             * stale单独统计。valid表示曾取得过完整样本，
             * stale表示这份last-good样本当前已经过期。
             */
            if (snapshot.imu_status.stale == RT_TRUE)
            {
                stats.stale_count++;
            }

            /*
             * 同时检查样本与状态中的配置版本。
             * 任何一个偏离冻结值，都说明本段存在配置语义混用。
             */
            if ((snapshot.imu.config_version !=
                    frozen_config_version) ||
                (snapshot.imu_status.config_version !=
                    frozen_config_version))
            {
                stats.config_mismatch_count++;
            }

            if (current_sample_valid == RT_TRUE)
            {
                /*
                 * 第一份有效样本建立序号和时间基准，
                 * 因为没有前一份样本，所以delta_tick约定为0。
                 */
                if (previous_sample_valid == RT_FALSE)
                {
                    is_new_sample = RT_TRUE;
                    delta_tick = 0U;
                }
                else
                {
                    /*
                     * 使用无符号序号减法识别重复和跳变，
                     * 在一次正常uint32回绕范围内仍可成立。
                     */
                    sequence_delta =
                        snapshot.imu_status.seq -
                        previous_imu_seq;

                    if (sequence_delta == 0U)
                    {
                        /* 序号未变化，本轮读到的是重复快照。 */
                        stats.duplicate_count++;
                    }
                    else
                    {
                        /*
                         * 序号变化表示取得新IMU样本。
                         * delta_tick使用两个样本自身的完成时间计算，
                         * 不能使用两个poll_tick代替。
                         */
                        is_new_sample = RT_TRUE;
                        delta_tick =
                            (rt_tick_t)(
                                snapshot.imu.sample_tick -
                                previous_sample_tick);

                        /*
                         * 差值大于1表示两份已观察样本之间存在序号缺口。
                         * 只累计缺失数量，不伪造缺失样本数据。
                         */
                        if (sequence_delta > 1U)
                        {
                            stats.seq_gap_count +=
                                sequence_delta - 1U;
                        }
                    }
                }

                /*
                 * 只有确认取得新有效样本时，才更新上一样本基准。
                 * 重复、无效、stale或快照失败均不能推进该基准。
                 */
                if (is_new_sample == RT_TRUE)
                {
                    stats.new_sample_count++;
                    previous_imu_seq =
                        snapshot.imu_status.seq;
                    previous_sample_tick =
                        snapshot.imu.sample_tick;
                    previous_sample_valid = RT_TRUE;
                }
            }
        }

        /*
         * 无论本轮成功还是失败都输出一行D记录。
         * 快照失败时传入RT_NULL，由输出函数生成明确的失败字段。
         */
        result = sensor_dataset_print_data(
            capture_index,
            poll_tick,
            late_ticks,
            result,
            is_new_sample,
            delta_tick,
            (result == RT_EOK) ? &snapshot : RT_NULL);

        if (result != RT_EOK)
        {
            rt_kprintf(
                "[DATASET][ERR] data output failed "
                "index=%lu result=%d\n",
                (unsigned long)capture_index,
                (int)result);

            command_result = result;
            break;
        }

        /*
         * capture_index只表示实际输出的数据行编号；
         * next_poll_tick按绝对周期推进，不依赖本轮实际完成时刻。
         */
        capture_index++;
        next_poll_tick += period_ticks;
    }

    /*
     * 最后一个计划采样点通常位于结束期限前一个周期。
     * 等待到正式结束期限后再输出#END，使elapsed_tick覆盖完整时长。
     */
    now = rt_tick_get();

    if (sensor_dataset_tick_reached(
            now,
            capture_end_deadline) == RT_FALSE)
    {
        rt_thread_delay(capture_end_deadline - now);
    }

    capture_end_tick = rt_tick_get();

    /*
     * 快照失败、无效、过期或配置不一致都会使本段返回失败。
     * 原始数据和#END统计仍然完整保留，便于定位问题和决定重采。
     */
    if ((stats.snapshot_error_count != 0U) ||
        (stats.invalid_count != 0U) ||
        (stats.stale_count != 0U) ||
        (stats.config_mismatch_count != 0U))
    {
        command_result = -RT_EIO;
    }

    /*
     * #END保存本段最终结果和完整质量统计。
     * 离线工具必须同时检查result及各计数，不能只看命令是否结束。
     */
    result = sensor_dataset_outputf(
        "#END,result=%d,poll_count=%lu,new_sample_count=%lu,"
        "duplicate_count=%lu,seq_gap_count=%lu,"
        "invalid_count=%lu,stale_count=%lu,"
        "snapshot_error_count=%lu,config_mismatch_count=%lu,"
        "late_poll_count=%lu,schedule_skip_count=%lu,"
        "max_late_ticks=%lu,first_tick=%lu,last_tick=%lu,"
        "capture_end_tick=%lu,elapsed_tick=%lu\n",
        (int)command_result,
        (unsigned long)stats.poll_count,
        (unsigned long)stats.new_sample_count,
        (unsigned long)stats.duplicate_count,
        (unsigned long)stats.seq_gap_count,
        (unsigned long)stats.invalid_count,
        (unsigned long)stats.stale_count,
        (unsigned long)stats.snapshot_error_count,
        (unsigned long)stats.config_mismatch_count,
        (unsigned long)stats.late_poll_count,
        (unsigned long)stats.schedule_skip_count,
        (unsigned long)stats.max_late_ticks,
        (unsigned long)stats.first_tick,
        (unsigned long)stats.last_tick,
        (unsigned long)capture_end_tick,
        (unsigned long)(
            capture_end_tick - capture_start_tick));

    sensor_dataset_release();

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DATASET][ERR] END output failed result=%d\n",
            (int)result);

        return result;
    }

    return command_result;
}


MSH_CMD_EXPORT(imu_dataset_capture,
                capture IMU CSV: segment split class duration_s);

#endif /* RT_USING_FINSH */
