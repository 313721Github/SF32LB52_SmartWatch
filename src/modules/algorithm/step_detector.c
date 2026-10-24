#include "step_detector.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 任务10冻结配置。
 *
 * 这些值已经通过设计集选择并完成首次独立验证。
 * 若修改任何参数，必须递增算法/配置版本，并把后续验证标记为
 * post_validation_iteration，不能覆盖首次独立验证结论。
 */
const step_detector_config_t STEP_DETECTOR_CONFIG_B0_TASK10_V1 =
{
    STEP_DETECTOR_ALGORITHM_B0,
    300000U,   /* high_threshold_ug */
    0U,        /* B0不使用low threshold */
    500U,      /* refractory_ms */
    0U,        /* B0不使用EWMA */
    200U,      /* warmup_ms */
    100U,      /* reset_gap_ms */
    STEP_DETECTOR_TASK10_TICK_HZ,
    STEP_DETECTOR_TASK10_CONFIG_VERSION
};

const step_detector_config_t STEP_DETECTOR_CONFIG_B1_TASK10_V1 =
{
    STEP_DETECTOR_ALGORITHM_B1,
    180000U,   /* high_threshold_ug */
    126000U,   /* low_threshold_ug */
    300U,      /* refractory_ms */
    120U,      /* ewma_tau_ms */
    200U,      /* warmup_ms */
    100U,      /* reset_gap_ms */
    STEP_DETECTOR_TASK10_TICK_HZ,
    STEP_DETECTOR_TASK10_CONFIG_VERSION
};

//公开函数声明
int step_detector_init(step_detector_t *detector,const step_detector_config_t *config); //初始化一个算法实例初始化会复制配置并清零运行状态
void step_detector_reset(step_detector_t *detector);    //清除一个实例的全部运行数据,保留已验证的配置，清除步数、滤波状态和诊断计数
step_detector_process_result_t step_detector_process(step_detector_t *detector,
                                                    const step_detector_input_t *input,
                                                    step_detector_output_t *output);    //处理一份IMU输入,本函数只执行有限整数计算，不延时、不打印、不访问硬件

//私有函数声明
static bool step_detector_config_is_valid(const step_detector_config_t *config);        //检查计步算法配置的公共字段、算法类型和字段之间的约束关系
static void step_detector_clear_output(const step_detector_t *detector,
                                        step_detector_output_t *output);                //清空一次处理调用的输出结构，并保留当前已经确认的累计步数
static bool step_detector_reset_phase(step_detector_t *detector);                       //重置计步算法的滤波、预热、滞回和事件间隔等阶段状态
static uint32_t step_detector_ticks_to_ms(const step_detector_t *detector,
                                            uint32_t tick_delta);                       //把两个系统tick之间的无符号差值换算为整数毫秒
static uint32_t step_detector_integer_sqrt_u64(uint64_t value);                         //计算64位无符号整数的整数平方根
static uint64_t step_detector_square_i32(int32_t value);                                //计算一个32位有符号加速度分量的平方
static uint32_t step_detector_calculate_magnitude(const step_detector_input_t *input);  //根据三轴加速度计算向下取整的加速度矢量模长
static uint32_t step_detector_calculate_dynamic(uint32_t magnitude_ug);                 //根据加速度模长计算相对于1g的绝对动态分量
static int64_t step_detector_round_divide_signed(int64_t numerator,int64_t denominator);//对有符号整数分子执行与符号对称的四舍五入除法
static uint32_t step_detector_update_ewma(uint32_t previous,uint32_t current,
                                            uint32_t dt_ms,uint32_t tau_ms);            //根据真实样本时间间隔更新B1使用的整数EWMA滤波状态
static void step_detector_start_phase(step_detector_t *detector,const step_detector_input_t *input,
                                        uint32_t dynamic_ug);                           //使用当前有效新样本建立一个新的连续处理阶段和预热起点


//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*
 * 简介：使用合法配置初始化计步算法上下文。
 * 输入：detector为调用者持有的上下文；config为只读配置。
 * 返回值：成功返回0；指针为空或配置非法时返回-1。
 * 说明：初始化会清除全部运行状态、累计步数和统计，并复制配置。
 *       第一份有效样本只建立预热阶段，不会立即计步。
 */
int step_detector_init(step_detector_t *detector,const step_detector_config_t *config)
{
    /* 输入非法时保留detector原有内容。 */
    if ((detector == NULL) ||!step_detector_config_is_valid(config))
    {
        return -1;
    }

    /* 建立确定的全零运行状态。 */
    (void)memset(detector, 0, sizeof(*detector));

    /* 配置按值复制，不依赖外部config的生命周期。 */
    detector->config = *config;
    detector->configured = true;
    detector->armed = true;

    return 0;
}


/*
 * 简介：保留配置，对算法实例执行完整复位。
 * 输入：detector为待复位上下文；NULL或未配置时不执行操作。
 * 返回值：无。
 * 说明：清除累计步数、序号、算法状态和统计，用于开始一轮新测试。
 *       与reset_phase不同，本函数不保留上一样本序号和累计结果。
 */
void step_detector_reset(step_detector_t *detector)
{
    step_detector_config_t config;

    if ((detector == NULL) ||!detector->configured)
    {
        return;
    }

    /* memset前保存已经验证过的配置。 */
    config = detector->config;

    /* 清除全部运行状态和累计统计。 */
    (void)memset(detector, 0, sizeof(*detector));

    /* 恢复配置，并回到等待第一份有效样本的状态。 */
    detector->config = config;
    detector->configured = true;
    detector->armed = true;
}

/*
 * 简介：处理一份IMU样本，并按冻结的B0或B1算法判断是否产生计步事件。
 * 输入：detector为算法实例；input为当前IMU样本；output接收本次计算结果。
 * 返回值：ERROR表示参数或配置错误；IGNORED表示样本被拒绝；
 *         ACCEPTED表示样本已处理但未计步；EVENT表示产生一次计步事件。
 * 说明：函数依次完成样本门禁、序号检查、特征计算、时间连续性检查、
 *       预热、阈值/滞回和最小事件间隔判断。它不访问传感器、线程或UI。
 */
step_detector_process_result_t step_detector_process(step_detector_t *detector,const step_detector_input_t *input,step_detector_output_t *output)
{
    uint32_t sequence_delta;
    uint32_t tick_delta;
    uint32_t dt_ms;
    uint32_t warmup_elapsed_ms;
    uint32_t event_elapsed_ms;
    uint32_t magnitude_ug;
    uint32_t dynamic_ug;
    bool refractory_ok;
    bool sequence_rollback = false;

    /* 参数错误时不能访问context、input或output。 */
    if ((detector == NULL) ||
        (input == NULL) ||
        (output == NULL))
    {
        return STEP_DETECTOR_PROCESS_ERROR;
    }

    step_detector_clear_output(detector, output);

    /* 未成功配置的实例不能参与计算。 */
    if (!detector->configured || !step_detector_config_is_valid(&detector->config))
    {
        return STEP_DETECTOR_PROCESS_ERROR;
    }

    /*
     * 无效、过期或配置不匹配的样本不能进入算法。
     * 同时清除本轮滤波和事件相位，避免恢复后误计步。
     */
    if (!input->valid)
    {
        detector->invalid_count++;

        if (step_detector_reset_phase(detector))
        {
            detector->reset_count++;
            output->state_reset = true;
        }

        output->reason = STEP_DETECTOR_REASON_INVALID;
        return STEP_DETECTOR_PROCESS_IGNORED;
    }

    if (input->stale)
    {
        detector->stale_count++;

        if (step_detector_reset_phase(detector))
        {
            detector->reset_count++;
            output->state_reset = true;
        }

        output->reason = STEP_DETECTOR_REASON_STALE;
        return STEP_DETECTOR_PROCESS_IGNORED;
    }

    if (input->config_version !=detector->config.expected_config_version)
    {
        detector->config_mismatch_count++;

        if (step_detector_reset_phase(detector))
        {
            detector->reset_count++;
            output->state_reset = true;
        }

        output->reason =STEP_DETECTOR_REASON_CONFIG_MISMATCH;

        return STEP_DETECTOR_PROCESS_IGNORED;
    }

    /*
     * 通过无符号差值识别重复、正向缺样和序号回退。
     * UINT32_MAX到0的自然回绕会得到delta=1，仍视为连续。
     */
    if (detector->has_previous_seq)
    {
        sequence_delta =input->imu_seq -detector->previous_imu_seq;

        if (sequence_delta == 0U)
        {
            detector->duplicate_count++;
            output->reason =STEP_DETECTOR_REASON_DUPLICATE;

            return STEP_DETECTOR_PROCESS_IGNORED;
        }

        if (sequence_delta < 0x80000000U)
        {
            if (sequence_delta > 1U)
            {
                detector->seq_gap_count +=sequence_delta - 1U;
            }
        }
        else
        {
            detector->sequence_rollback_count++;
            detector->reset_count++;

            (void)step_detector_reset_phase(detector);

            sequence_rollback = true;
            output->state_reset = true;
        }
    }

    /*
     * valid且非重复的样本在这里成为最新已接受序号。
     * 即使后面触发时间重置，也不会再次处理同一份样本。
     */
    detector->previous_imu_seq = input->imu_seq;
    detector->has_previous_seq = true;

    /* 计算模长以及相对1 g的动态分量。 */
    magnitude_ug =step_detector_calculate_magnitude(input);

    dynamic_ug =step_detector_calculate_dynamic(magnitude_ug);

    output->magnitude_ug = magnitude_ug;
    output->dynamic_ug = dynamic_ug;
    output->sample_accepted = true;

    detector->accepted_count++;

    /*
     * 第一份样本或序号回退后的样本只建立预热起点，
     * 不能直接产生计步事件。
     */
    if (!detector->initialized || sequence_rollback)
    {
        step_detector_start_phase(
            detector,
            input,
            dynamic_ug);

        output->filtered_dynamic_ug =detector->filtered_dynamic_ug;

        output->reason = sequence_rollback
                ? STEP_DETECTOR_REASON_SEQUENCE_ROLLBACK_RESET
                : STEP_DETECTOR_REASON_INITIALIZED;

        return STEP_DETECTOR_PROCESS_ACCEPTED;
    }

    tick_delta =input->tick -detector->previous_tick;

    dt_ms = step_detector_ticks_to_ms(detector,tick_delta);

    /*
     * 样本间隔过大时重新开始预热。
     * 当前边界样本只作为新相位起点，不参与事件判断。
     */
    if (dt_ms > detector->config.reset_gap_ms)
    {
        (void)step_detector_reset_phase(detector);
        detector->reset_count++;

        step_detector_start_phase(
            detector,
            input,
            dynamic_ug);

        output->filtered_dynamic_ug =detector->filtered_dynamic_ug;

        output->state_reset = true;
        output->reason =
            STEP_DETECTOR_REASON_TIME_GAP_RESET;

        return STEP_DETECTOR_PROCESS_ACCEPTED;
    }

    detector->previous_tick = input->tick;

    /* B1使用时间型EWMA；B0直接使用当前动态量。 */
    if (detector->config.algorithm ==STEP_DETECTOR_ALGORITHM_B1)
    {
        detector->filtered_dynamic_ug =step_detector_update_ewma(
                                        detector->filtered_dynamic_ug,
                                        dynamic_ug,
                                        dt_ms,
                                        detector->config.ewma_tau_ms);
    }
    else
    {
        detector->filtered_dynamic_ug = dynamic_ug;
    }

    output->filtered_dynamic_ug =detector->filtered_dynamic_ug;

    /*
     * 预热期间允许滤波器收敛，但禁止产生事件。
     */
    warmup_elapsed_ms =step_detector_ticks_to_ms(detector,input->tick -detector->warmup_start_tick);

    if (warmup_elapsed_ms <
        detector->config.warmup_ms)
    {
        output->reason =
            STEP_DETECTOR_REASON_ACCEPTED;

        return STEP_DETECTOR_PROCESS_ACCEPTED;
    }

    /*
     * refractory限制相邻计步事件的最短间隔。
     * 尚未产生过事件时不受该条件限制。
     */
    refractory_ok = !detector->has_last_event;

    if (detector->has_last_event)
    {
        event_elapsed_ms =step_detector_ticks_to_ms(detector,input->tick -detector->last_event_tick);

        refractory_ok =event_elapsed_ms >=detector->config.refractory_ms;
    }

    /* B0：单阈值加最小事件间隔。 */
    if (detector->config.algorithm ==STEP_DETECTOR_ALGORITHM_B0)
    {
        if ((dynamic_ug >=detector->config.high_threshold_ug) && refractory_ok)
        {
            detector->step_count++;
            detector->last_event_tick = input->tick;
            detector->has_last_event = true;

            output->event_generated = true;
            output->event_tick = input->tick;
            output->total_step_count =
                detector->step_count;
            output->reason =STEP_DETECTOR_REASON_EVENT;

            return STEP_DETECTOR_PROCESS_EVENT;
        }
    }
    else
    {
        /*
         * B1：armed状态越过高阈值时计步并解除armed；
         * 只有回落到低阈值后才重新armed，形成滞回。
         */
        if (detector->armed &&
            (detector->filtered_dynamic_ug >=
             detector->config.high_threshold_ug) &&
            refractory_ok)
        {
            detector->step_count++;
            detector->last_event_tick = input->tick;
            detector->has_last_event = true;
            detector->armed = false;

            output->event_generated = true;
            output->event_tick = input->tick;
            output->total_step_count =
                detector->step_count;
            output->reason =
                STEP_DETECTOR_REASON_EVENT;

            return STEP_DETECTOR_PROCESS_EVENT;
        }

        if (!detector->armed &&
            (detector->filtered_dynamic_ug <=
             detector->config.low_threshold_ug))
        {
            detector->armed = true;
        }
    }

    output->reason = STEP_DETECTOR_REASON_ACCEPTED;
    output->total_step_count = detector->step_count;

    return STEP_DETECTOR_PROCESS_ACCEPTED;
}

//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：检查计步算法配置的公共字段、算法类型和字段之间的约束关系。
 * 输入：config为待检查的计步算法配置，只读借用，函数不会修改其内容。
 * 返回值：配置完整且字段关系合法时返回true；
 *         config为空、公共字段为0、算法类型未知或算法专用字段非法时
 *         返回false。
 * 说明：B0不使用低阈值和EWMA时间常数，因此这两个字段必须为0；
 *       B1必须满足0 < low_threshold_ug < high_threshold_ug，
 *       并且ewma_tau_ms必须大于0。tick频率、预期IMU配置版本、
 *       高阈值、预热时间、长缺口门限和最小事件间隔是两种算法
 *       共同需要的字段，均不得为0。本函数只验证配置结构关系，
 *       不证明这些参数已经与冻结文件或离线算法结果一致。
 */
static bool step_detector_config_is_valid(const step_detector_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if ((config->tick_hz == 0U) ||
        (config->expected_config_version == 0U) ||
        (config->high_threshold_ug == 0U) ||
        (config->warmup_ms == 0U) ||
        (config->reset_gap_ms == 0U) ||
        (config->refractory_ms == 0U))
    {
        return false;
    }

    if (config->algorithm == STEP_DETECTOR_ALGORITHM_B0)
    {
        return ((config->low_threshold_ug == 0U) &&
                (config->ewma_tau_ms == 0U));
    }

    if (config->algorithm == STEP_DETECTOR_ALGORITHM_B1)
    {
        return ((config->low_threshold_ug > 0U) &&
                (config->low_threshold_ug <
                    config->high_threshold_ug) &&
                (config->ewma_tau_ms > 0U));
    }

    return false;
}

/*
 * 简介：清空一次处理调用的输出结构，并保留当前已经确认的累计步数。
 * 输入：detector为计步算法上下文，只读借用；允许为NULL。
 *       output为调用者提供的输出缓冲区，必须为有效非NULL指针。
 * 返回值：无。
 * 说明：函数先把output的全部字段清零，再把reason初始化为
 *       STEP_DETECTOR_REASON_NONE。detector有效时，将当前step_count
 *       复制到total_step_count，使忽略、重置或错误路径仍能返回调用前
 *       已确认的累计结果。本函数不会修改detector，也不会产生计步事件。
 *       output的非NULL条件由调用该私有函数的上层入口统一保证。
 */
static void step_detector_clear_output(const step_detector_t *detector,step_detector_output_t *output)
{
    (void)memset(output, 0, sizeof(*output));

    output->reason = STEP_DETECTOR_REASON_NONE;

    if (detector != NULL)
    {
        output->total_step_count = detector->step_count;
    }
}

/*
 * 简介：重置计步算法的滤波、预热、滞回和事件间隔等阶段状态。
 * 输入：detector为需要重置阶段状态的计步算法上下文，必须为有效的
 *       非NULL指针。
 * 返回值：重置前存在已初始化状态、历史事件或未重新armed的滞回状态时
 *         返回true；上下文本来已经处于干净的未初始化阶段时返回false。
 * 说明：本函数只清除与当前连续采样阶段有关的临时状态，保留累计步数、
 *       诊断统计、has_previous_seq和previous_imu_seq。保留上一样本序号
 *       可以避免器件失败或恢复后，把同一份last-good样本再次识别为新样本。
 *       返回值用于决定是否把本次操作计入reset_count，避免连续收到多份
 *       无效或过期输入时重复统计同一次状态重置。本函数不检查detector
 *       是否为NULL，调用者必须在进入本私有函数前完成参数检查。
 */
static bool step_detector_reset_phase(step_detector_t *detector)
{
    bool had_active_state;

    had_active_state =detector->initialized || detector->has_last_event || !detector->armed;

    detector->initialized = false;
    detector->armed = true;
    detector->has_last_event = false;

    detector->previous_tick = 0U;
    detector->warmup_start_tick = 0U;
    detector->last_event_tick = 0U;
    detector->filtered_dynamic_ug = 0U;

    return had_active_state;
}

/*
 * 简介：把两个系统tick之间的无符号差值换算为整数毫秒。
 * 输入：detector为已经完成合法配置的计步算法上下文，只读借用；
 *       tick_delta为通过无符号tick减法得到的时间差，单位为tick。
 * 返回值：换算后的整数毫秒数；不能整除的部分向下取整。
 *         如果计算结果超过uint32_t可表示范围，则返回UINT32_MAX。
 * 说明：乘法先提升为uint64_t，避免tick_delta乘以1000时发生32位溢出。
 *       tick_delta应由“当前tick减去上一tick”的无符号减法得到，因此能够
 *       处理一次正常的32位tick回绕。该函数只完成单位换算，不判断时间
 *       缺口是否超过reset_gap_ms，也不验证detector或tick_hz；调用者必须
 *       保证detector非NULL且配置已经通过检查。换算使用整数除法，余数
 *       向下舍弃，这一语义必须与Python离线回放保持一致。
 */
static uint32_t step_detector_ticks_to_ms(const step_detector_t *detector,uint32_t tick_delta)
{
    uint64_t milliseconds;

    milliseconds = ((uint64_t)tick_delta * 1000ULL) / (uint64_t)detector->config.tick_hz;

    if (milliseconds > UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)milliseconds;
}

/*
 * 简介：计算64位无符号整数的整数平方根。
 * 输入：value为需要计算平方根的64位无符号整数。
 * 返回值：返回floor(sqrt(value))，即不大于真实平方根的最大整数；结果范围为0～UINT32_MAX。
 * 说明：本函数使用逐位整数平方根算法，不使用浮点数、sqrt()或libm，
 *       从而使目标板C实现与Python math.isqrt()保持相同的向下取整语义。
 *       bit从不超过uint64_t范围的最高偶数位幂开始，每轮右移2位，
 *       因此循环次数具有固定上界，不会形成无限循环。uint64_t最大值
 *       的平方根不超过UINT32_MAX，所以最终转换为uint32_t不会截断。
 */
static uint32_t step_detector_integer_sqrt_u64( uint64_t value)
{
    uint64_t remainder = value;
    uint64_t result = 0ULL;
    uint64_t bit = 1ULL << 62;

    while (bit > remainder)
    {
        bit >>= 2;
    }

    while (bit != 0ULL)
    {
        if (remainder >= (result + bit))
        {
            remainder -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }

        bit >>= 2;
    }

    return (uint32_t)result;
}


/*
 * 简介：计算一个32位有符号加速度分量的平方。
 * 输入：value为任意int32_t数值，当前算法中表示某一轴的加速度，单位为µg。
 * 返回值：返回value的精确平方，类型为uint64_t。
 * 说明：函数先把value扩展为int64_t，再执行乘法，避免在32位类型中
 *       发生有符号乘法溢出。即使value等于INT32_MIN，其平方为2^62，
 *       仍能安全保存在int64_t和uint64_t中。本函数不进行单位换算，
 *       返回值的量纲为µg²。
 */
static uint64_t step_detector_square_i32(int32_t value)
{
    int64_t wide_value = (int64_t)value;

    return (uint64_t)(wide_value * wide_value);
}


/*
 * 简介：根据三轴加速度计算向下取整的加速度矢量模长。
 * 输入：input为一份计步算法输入，只读借用；必须为有效非NULL指针。
 *       其中ax_ug、ay_ug和az_ug的单位均为µg。
 * 返回值：返回floor(sqrt(ax²+ay²+az²))，单位为µg。
 * 说明：每个轴先使用64位中间值计算平方，再进行三轴求和，防止32位
 *       乘法和加法溢出。三个任意int32_t分量的平方和仍可保存在
 *       uint64_t中。最终调用整数平方根函数，保证与Python离线回放的
 *       math.isqrt()语义一致。本函数不检查input是否为NULL，调用者
 *       必须在调用前完成参数检查。
 */
static uint32_t step_detector_calculate_magnitude(const step_detector_input_t *input)
{
    uint64_t square_sum;

    square_sum =step_detector_square_i32(input->ax_ug) +
                step_detector_square_i32(input->ay_ug) +
                step_detector_square_i32(input->az_ug);

    return step_detector_integer_sqrt_u64(square_sum);
}


/*
 * 简介：根据加速度模长计算相对于1g的绝对动态分量。
 * 输入：magnitude_ug为三轴加速度模长，单位为µg。
 * 返回值：返回abs(magnitude_ug-1000000)，单位为µg。
 * 说明：1g固定表示为1000000 µg。函数通过大小比较后执行无符号减法，
 *       避免直接相减产生下溢，也不依赖有符号abs()。该结果只表示
 *       加速度模长偏离1g的程度，无法单独区分步行、挥手、敲击或其他
 *       非目标运动。
 */
static uint32_t step_detector_calculate_dynamic(
    uint32_t magnitude_ug)
{
    const uint32_t gravity_ug = 1000000U;

    if (magnitude_ug >= gravity_ug)
    {
        return magnitude_ug - gravity_ug;
    }

    return gravity_ug - magnitude_ug;
}

/*
 * 简介：对有符号整数分子执行与符号对称的四舍五入除法。
 * 输入：numerator为待除的有符号分子；denominator为有符号分母，必须大于0。
 * 返回值：denominator大于0时，返回numerator/denominator经过四舍五入后的整数结果；denominator小于或等于0时返回0。
 * 说明：正数和负数分别对绝对值增加denominator/2后再执行整数除法，
 *       从而避免直接依赖C语言负数除法向零截断的结果。该语义必须与
 *       Python rounded_divide_signed()完全一致。调用者必须保证
 *       numerator不等于INT64_MIN，否则对其执行取负会产生有符号溢出。
 *       当前EWMA调用路径在dt_ms不超过reset_gap_ms时满足该范围约束。
 */
static int64_t step_detector_round_divide_signed( int64_t numerator,int64_t denominator)
{
    if (denominator <= 0)
    {
        return 0;
    }

    if (numerator >= 0)
    {
        return(numerator + (denominator / 2)) /denominator;
    }

    return-(((-numerator) + (denominator / 2)) /denominator);
}


/*
 * 简介：根据真实样本时间间隔更新B1使用的整数EWMA滤波状态。
 * 输入：previous为上一时刻的滤波结果，单位为µg；
 *       current为当前样本的dynamic值，单位为µg；
 *       dt_ms为当前样本与上一有效样本的时间间隔，单位为ms；
 *       tau_ms为冻结的EWMA时间常数，单位为ms且必须大于0。
 * 返回值：返回本次更新后的EWMA结果，单位为µg；dt_ms为0时直接返回
 *         previous；计算结果超出uint32_t范围时执行饱和处理。
 * 说明：使用公式
 *       filtered += round_signed((current-filtered)*dt_ms/(tau_ms+dt_ms))
 *       进行按真实时间间隔更新。差值、乘法、分母和调整量均使用int64_t，
 *       避免32位运算溢出。正常EWMA结果应位于previous和current之间，
 *       末尾的0与UINT32_MAX限制是防御性保护。本函数不负责判断长时间
 *       缺口；调用者必须先比较dt_ms和reset_gap_ms，超过门限时应重置
 *       阶段，不能把过大的dt直接用于EWMA。
 */
static uint32_t step_detector_update_ewma(uint32_t previous,uint32_t current,uint32_t dt_ms,uint32_t tau_ms)
{
    int64_t difference;
    int64_t numerator;
    int64_t denominator;
    int64_t adjustment;
    int64_t updated;

    if (dt_ms == 0U)
    {
        return previous;
    }

    difference =(int64_t)current -(int64_t)previous;

    numerator =difference *(int64_t)dt_ms;

    denominator =(int64_t)tau_ms + (int64_t)dt_ms;

    adjustment = step_detector_round_divide_signed(numerator,denominator);

    updated =(int64_t)previous + adjustment;

    if (updated <= 0)
    {
        return 0U;
    }

    if (updated >= (int64_t)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)updated;
}


/*
 * 简介：使用当前有效新样本建立一个新的连续处理阶段和预热起点。
 * 输入：detector为需要建立阶段状态的计步算法上下文，必须为有效的
 *       非NULL指针；
 *       input为当前已经通过有效性、配置和序号门禁的输入样本，
 *       必须为有效的非NULL指针；
 *       dynamic_ug为当前样本计算得到的动态加速度，单位为µg。
 * 返回值：无。
 * 说明：本函数把当前样本的tick同时记录为上一有效样本时间和预热起点，
 *       并用当前dynamic直接初始化滤波器，不执行EWMA过渡。新阶段从
 *       armed状态开始，同时清除上一阶段的事件间隔状态。当前样本只用于
 *       建立滤波、时间和滞回初值，不能在初始化、恢复或长时间缺口边界
 *       立即产生计步事件。函数不修改累计步数、累计诊断统计和上一IMU
 *       序号，这些字段由上层处理流程统一维护。
 */
static void step_detector_start_phase(step_detector_t *detector,const step_detector_input_t *input,uint32_t dynamic_ug)
{
    detector->initialized = true;
    detector->armed = true;
    detector->has_last_event = false;

    detector->previous_tick = input->tick;
    detector->warmup_start_tick = input->tick;
    detector->last_event_tick = 0U;

    detector->filtered_dynamic_ug = dynamic_ug;
}
