#include "rtthread.h"
#include "step_detector.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#include "step_detector_golden_vectors.h"
#include <stdint.h>
#include "sensor_service.h"
#include <stdlib.h>

/**
 * @brief 构造值自测累计结果
 *
 * 该结构只在FinSH命令栈中使用，不进入sensor服务，也不参与算法状态。
 */
typedef struct
{
    rt_uint32_t total_count;
    rt_uint32_t pass_count;
    rt_uint32_t fail_count;
} step_detector_test_stats_t;
/*
 * 实际事件最多保存8个。
 * 当前黄金窗口预期每种算法只有3个事件，额外容量用于安全捕获异常事件，
 * 同时避免在FinSH线程栈中创建过大的数组。
 */
#define STEP_GOLDEN_ACTUAL_EVENT_CAPACITY 8U

/**
 * @brief 黄金向量测试累计结果
 *
 * 所有字段只属于本次FinSH命令，不进入算法或sensor服务状态。
 */
typedef struct
{
    rt_uint32_t sample_count;
    rt_uint32_t process_mismatch_count;
    rt_uint32_t feature_mismatch_count;

    rt_uint32_t b0_actual_event_count;
    rt_uint32_t b0_event_overflow_count;
    rt_uint32_t b0_event_mismatch_count;

    rt_uint32_t b1_actual_event_count;
    rt_uint32_t b1_event_overflow_count;
    rt_uint32_t b1_event_mismatch_count;

    rt_uint32_t final_state_mismatch_count;
} step_detector_golden_stats_t;

#define STEP_ONLINE_DEFAULT_DURATION_S       30U
#define STEP_ONLINE_MIN_DURATION_S            5U
#define STEP_ONLINE_MAX_DURATION_S          180U
#define STEP_ONLINE_POLL_MS                   10U
#define STEP_ONLINE_MAX_REFERENCE_STEPS     1000U

/**
 * @brief 有限在线对照命令的本地统计
 *
 * 该结构只属于当前FinSH命令，不进入sensor服务或计步算法状态。
 * 在线命令退出后所有字段失效。
 */
typedef struct
{
    rt_uint32_t poll_count;
    rt_uint32_t snapshot_success_count;
    rt_uint32_t snapshot_error_count;

    rt_uint32_t new_imu_seq_count;
    rt_uint32_t duplicate_snapshot_count;

    rt_uint32_t valid_input_count;
    rt_uint32_t invalid_input_count;
    rt_uint32_t stale_input_count;
    rt_uint32_t snapshot_contract_mismatch_count;

    rt_uint32_t process_error_count;
    rt_tick_t max_process_ticks;
} step_detector_online_stats_t;
#define STEP_TEST_EXPECT(stats, condition, name) step_detector_test_expect((stats),(condition) ? RT_TRUE : RT_FALSE,(name))

static void step_detector_test_expect(step_detector_test_stats_t *stats,
                                    rt_bool_t passed, const char *name);    //记录一项计步算法构造值自测断言，并输出对应的PASS或FAIL结果
static step_detector_input_t step_detector_test_make_input(rt_uint32_t tick,
                                                        rt_uint32_t imu_seq,
                                                        rt_int32_t ax_ug,
                                                        rt_int32_t ay_ug,
                                                        rt_int32_t az_ug);  //根据指定的时间、序号和三轴加速度构造一份合法算法输入
static step_detector_input_t step_detector_test_make_dynamic_input(rt_uint32_t tick,
                                rt_uint32_t imu_seq,rt_uint32_t dynamic_ug);//构造一份具有指定dynamic值、计算结果容易人工验证的单轴输入
static void step_detector_test_guards(step_detector_test_stats_t *stats);   //验证计步算法初始化配置门禁和公开处理接口的空指针保护
static void step_detector_test_magnitude(step_detector_test_stats_t *stats);//验证三轴整数模长、重力动态量、符号扩展和算法首样本处理结果
static void step_detector_test_b0(step_detector_test_stats_t *stats);       //验证B0算法的200 ms预热、阈值等号和500 ms最小事件间隔
static void step_detector_test_b1(step_detector_test_stats_t *stats);       //验证B1算法的整数EWMA、高低阈值滞回和300 ms最小事件间隔
static void step_detector_test_input_contract(step_detector_test_stats_t *stats);//验证重复序号、正向缺样、长时间缺口、无效/过期样本和配置错误门禁
static void step_detector_test_wraparound(step_detector_test_stats_t *stats);   //验证32位系统tick和IMU样本序号的自然回绕及异常倒退处理
static int step_detector_self_test(int argc,char **argv);                   //依次执行计步算法全部构造值测试，并输出最终通过与失败统计

static rt_uint32_t step_detector_golden_compare_events(const char *algorithm,
                                    const step_detector_golden_event_t *actual,
                                    rt_uint32_t actual_count,
                                    rt_uint32_t overflow_count,
                                    const step_detector_golden_event_t *expected,
                                    rt_uint32_t expected_count);            //比较一种目标板C算法产生的实际事件与Python生成的黄金事件

static int step_detector_golden_test(int argc,char **argv);                 //使用真实设计集黄金向量验证目标板C算法
static rt_uint32_t step_detector_online_abs_diff(rt_uint32_t actual,
                                                rt_uint32_t expected);      //计算在线输出与人工参考值的绝对误差
static int step_detector_online_test(int argc, char **argv);                //有限时间读取sensor快照并同时运行B0和B1
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：记录一项计步算法构造值自测断言，并输出对应的PASS或FAIL结果。
 * 输入：stats为当前自测过程的累计统计，由调用者持有并保证非NULL；
 *       passed表示断言是否成立；name为描述测试边界的只读字符串。
 * 返回值：无。
 * 说明：每次调用都会增加total_count，并根据passed增加pass_count或
 *       fail_count。函数只负责测试统计和UART输出，不修改计步算法上下文，
 *       不访问传感器、sensor服务或LVGL对象。name必须在函数调用期间有效。
 */
static void step_detector_test_expect(step_detector_test_stats_t *stats,rt_bool_t passed,const char *name)
{
    /* 每执行一项断言，都先累计测试总数。 */
    stats->total_count++;

    if (passed)
    {
        /* 断言成立，记录通过数量并输出可检索的PASS标记。 */
        stats->pass_count++;

        rt_kprintf("[STEP][SELFTEST][PASS] %s\n",name);
    }
    else
    {
        /*
         * 断言失败时继续执行剩余测试，以便一次UART日志能够暴露
         * 多个相互独立的问题，最终由汇总结果统一判定失败。
         */
        stats->fail_count++;

        rt_kprintf("[STEP][SELFTEST][FAIL] %s\n",name);
    }
}


/*
 * 简介：根据指定的时间、序号和三轴加速度构造一份合法算法输入。
 * 输入：tick为样本完成时刻；imu_seq为IMU样本序号；ax_ug、ay_ug和
 *       az_ug分别为三轴加速度，单位均为µg。
 * 返回值：返回字段完整的step_detector_input_t结构体副本。
 * 说明：构造值默认使用任务10冻结的IMU配置版本，并设置valid=true、
 *       stale=false，使样本能够进入正常算法路径。函数不读取真实快照，
 *       不保存输入结构地址，调用者可在返回后修改字段以构造失败场景。
 */
static step_detector_input_t step_detector_test_make_input( rt_uint32_t tick,rt_uint32_t imu_seq,
                                                            rt_int32_t ax_ug,
                                                            rt_int32_t ay_ug,
                                                            rt_int32_t az_ug)
{
    /*
     * 先清零整个结构，确保将来结构增加字段时，未显式赋值的字段
     * 仍具有确定初值，不把当前栈中的不确定内容带入算法。
     */
    step_detector_input_t input = {0};

    /* 时间和序号用于验证预热、缺口、重复、回绕与倒退处理。 */
    input.tick = tick;
    input.imu_seq = imu_seq;

    /* 三轴统一使用µg，保持与sensor_imu_sample_t的物理量语义一致。 */
    input.ax_ug = ax_ug;
    input.ay_ug = ay_ug;
    input.az_ug = az_ug;

    /*
     * 正常构造值使用任务10冻结配置版本。
     * 配置不匹配测试可以在调用本函数后单独覆盖该字段。
     */
    input.config_version =STEP_DETECTOR_TASK10_CONFIG_VERSION;

    /* 默认生成一份新的、有效且未过期的算法输入。 */
    input.valid = true;
    input.stale = false;

    return input;
}


/*
 * 简介：构造一份具有指定dynamic值、计算结果容易人工验证的单轴输入。
 * 输入：tick为样本完成时刻；imu_seq为IMU样本序号；dynamic_ug为期望的
 *       重力模长偏差，单位为µg。
 * 返回值：返回X轴为1g加dynamic、Y/Z轴为0的合法算法输入。
 * 说明：该构造使模长等于1000000 + dynamic_ug，因此算法计算得到的
 *       abs(magnitude-1000000)应严格等于dynamic_ug，适合验证阈值、
 *       EWMA和状态机。当前测试值必须保证加法结果不超过int32_t范围。
 */
static step_detector_input_t step_detector_test_make_dynamic_input(rt_uint32_t tick,rt_uint32_t imu_seq,rt_uint32_t dynamic_ug)
{
    /*
     * 仅在X轴加入重力基准和动态量，使预期模长无需浮点运算即可
     * 人工推导，避免测试数据本身引入额外的平方根误差。
     */
    return step_detector_test_make_input(tick,imu_seq,
        (rt_int32_t)(1000000U + dynamic_ug),
        0,0);
}


/*
 * 简介：验证计步算法初始化配置门禁和公开处理接口的空指针保护。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：先分别构造非法B0和B1配置，确认初始化拒绝字段关系错误的配置；
 *       再使用任务10冻结B1配置建立合法上下文，依次传入空detector、
 *       空input和空output。测试只检查防御性边界，不执行传感器访问，
 *       也不改变全局sensor服务、数据集或UI状态。
 */
static void step_detector_test_guards(step_detector_test_stats_t *stats)
{
    step_detector_t detector;
    step_detector_config_t config;
    step_detector_input_t input;
    step_detector_output_t output;
    int result;

    /*
     * B0明确不使用低阈值。把low_threshold_ug改为1，
     * 用于确认配置检查不会默默接受带有歧义的B0参数。
     */
    config = STEP_DETECTOR_CONFIG_B0_TASK10_V1;
    config.low_threshold_ug = 1U;

    result = step_detector_init( &detector,&config);

    STEP_TEST_EXPECT(stats,result != 0,"B0 rejects nonzero low threshold");

    /*
     * B1低阈值必须严格小于高阈值，否则无法形成有效滞回区间。
     */
    config = STEP_DETECTOR_CONFIG_B1_TASK10_V1;
    config.low_threshold_ug =config.high_threshold_ug;

    result = step_detector_init(&detector,&config);

    STEP_TEST_EXPECT(stats,result != 0,"B1 rejects low threshold >= high threshold");

    /*
     * 非法配置测试结束后，用冻结B1配置建立合法上下文。
     * 后续空指针测试不应受到未初始化context的干扰。
     */
    result = step_detector_init( &detector,&STEP_DETECTOR_CONFIG_B1_TASK10_V1);

    STEP_TEST_EXPECT(stats,result == 0,"B1 frozen config accepted");

    /* 构造一份合法输入，单独改变各公开接口指针。 */
    input = step_detector_test_make_dynamic_input(0U,1U,0U);

    /*
     * detector为空时，函数必须立即返回ERROR，不能读取context，
     * 也不能访问input和output之外的不确定内存。
     */
    STEP_TEST_EXPECT(stats,
                    step_detector_process(
                        RT_NULL,
                        &input,
                        &output) == STEP_DETECTOR_PROCESS_ERROR,
                    "null detector rejected");

    /* input为空时不能计算模长或更新任何算法运行状态。 */
    STEP_TEST_EXPECT(stats,
                    step_detector_process(
                        &detector,
                        RT_NULL,
                        &output) == STEP_DETECTOR_PROCESS_ERROR,
                    "null input rejected");

    /*
     * output为空时不能写入结果。该调用同样必须在任何状态更新前
     * 返回ERROR，避免调用者无法取得结果却改变算法状态。
     */
    STEP_TEST_EXPECT(stats,
                    step_detector_process(
                        &detector,
                        &input,
                        RT_NULL) == STEP_DETECTOR_PROCESS_ERROR,
                    "null output rejected");
}

/*
 * 简介：验证三轴整数模长、重力动态量、符号扩展和算法首样本处理结果。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：使用300000、400000、0构造经典3-4-5向量，预期整数模长为
 *       500000 µg、dynamic为500000 µg；随后把X/Y轴改成负数重新测试，
 *       证明平方计算不会因有符号输入而改变模长。每轮首样本只初始化
 *       算法阶段，不应产生计步事件。本函数不访问真实IMU或sensor服务。
 */
static void step_detector_test_magnitude(step_detector_test_stats_t *stats)
{
    step_detector_t detector;
    step_detector_input_t input;
    step_detector_output_t output;
    step_detector_process_result_t result;

    /*
     * 使用B1冻结配置初始化context。
     * 此处只验证公共特征计算，选择B0或B1不会改变首样本模长。
     */
    STEP_TEST_EXPECT(stats,step_detector_init( &detector,&STEP_DETECTOR_CONFIG_B1_TASK10_V1) == 0,
        "magnitude case init");

    /*
     * 300000² + 400000² = 500000²。
     * 三轴模长应为500000 µg，与1g的距离也应为500000 µg。
     */
    input = step_detector_test_make_input(100U,1U,300000,400000,0);

    result = step_detector_process(&detector, &input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.reason == STEP_DETECTOR_REASON_INITIALIZED) &&
                    (output.magnitude_ug == 500000U) &&
                    (output.dynamic_ug == 500000U) &&
                    (output.filtered_dynamic_ug == 500000U) &&
                    !output.event_generated,
                    "3-4-5 magnitude and dynamic");

    /*
     * 完整复位运行状态和统计，但保留已经验证的冻结配置，
     * 使负轴样本仍作为一个新阶段的第一份输入。
     */
    step_detector_reset(&detector);

    /*
     * 改变轴方向但保持绝对值不变。
     * 若平方前的类型扩展正确，模长结果必须与正轴测试完全一致。
     */
    input = step_detector_test_make_input(100U,1U,-300000,-400000,0);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.magnitude_ug == 500000U) &&
                    (output.dynamic_ug == 500000U),
                    "negative axes preserve magnitude");
}


/*
 * 简介：验证B0算法的200 ms预热、阈值等号和500 ms最小事件间隔。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：全部样本的dynamic固定为300000 µg，正好等于B0高阈值。
 *       0 ms样本建立阶段，100 ms仍处于预热，200 ms产生首次事件；
 *       随后持续保持越阈状态，验证事件间隔不足500 ms时不会重复计步，
 *       并在700 ms边界产生第二个事件。相邻样本间隔均不超过100 ms，
 *       因而不会误触发长缺口重置。
 */
static void step_detector_test_b0(step_detector_test_stats_t *stats)
{
    /*
     * 首次事件位于200 ms。
     * 以下时刻与首次事件的间隔都小于500 ms，均不得再次计步。
     * 同时相邻输入最大间隔为100 ms，不超过reset_gap_ms。
     */
    static const rt_uint32_t quiet_ticks[] =
    {
        300U,
        400U,
        500U,
        600U,
        699U
    };

    step_detector_t detector;
    step_detector_input_t input;
    step_detector_output_t output;
    step_detector_process_result_t result;
    rt_bool_t quiet_ok = RT_TRUE;
    rt_size_t index;
    rt_uint32_t sequence = 1U;

    /* 使用任务10冻结的B0配置建立独立算法实例。 */
    STEP_TEST_EXPECT(stats,
                    step_detector_init(
                        &detector,
                        &STEP_DETECTOR_CONFIG_B0_TASK10_V1) == 0,
                    "B0 init");

    /*
     * 第一份样本位于0 ms，只建立序号、时间和预热起点。
     * 即使dynamic已经达到阈值，也不能立即产生事件。
     */
    input = step_detector_test_make_dynamic_input(0U,sequence++,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    !output.event_generated,
                    "B0 first sample only initializes phase");

    /*
     * 100 ms尚未达到200 ms预热门限。
     * 样本可以参与状态更新，但不能计步。
     */
    input = step_detector_test_make_dynamic_input(100U,sequence++,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    !output.event_generated,
                    "B0 cannot trigger before 200 ms warmup");

    /*
     * 200 ms恰好达到预热边界，且dynamic恰好等于高阈值。
     * 算法使用“>=”判断，因此应产生第一步事件。
     */
    input = step_detector_test_make_dynamic_input(200U,sequence++,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_EVENT) &&
                    output.event_generated &&
                    (output.event_tick == 200U) &&
                    (output.total_step_count == 1U),
                    "B0 threshold equality triggers after warmup");

    /*
     * 持续越过高阈值时，B0仍必须遵守500 ms最小事件间隔。
     * 循环继续运行全部样本，以便一次日志暴露所有潜在重复计步。
     */
    for (index = 0U;index < sizeof(quiet_ticks) / sizeof(quiet_ticks[0]);index++)
    {
        input = step_detector_test_make_dynamic_input(quiet_ticks[index],sequence++,300000U);

        result = step_detector_process(&detector,&input,&output);

        if ((result != STEP_DETECTOR_PROCESS_ACCEPTED) || output.event_generated ||
            (output.total_step_count != 1U))
        {
            quiet_ok = RT_FALSE;
        }
    }

    STEP_TEST_EXPECT(stats,
                    quiet_ok,
                    "B0 blocks repeated event before 500 ms");

    /*
     * 700 ms与首次事件200 ms正好相差500 ms，
     * 应通过最小事件间隔检查并产生第二个事件。
     */
    input = step_detector_test_make_dynamic_input(700U,sequence++,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_EVENT) &&
                    (output.event_tick == 700U) &&
                    (output.total_step_count == 2U),
                    "B0 permits event at 500 ms boundary");
}


/*
 * 简介：验证B1算法的整数EWMA、高低阈值滞回和300 ms最小事件间隔。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：先用0和500000 µg的dynamic输入验证tau=120 ms、dt=100 ms时
 *       的整数EWMA结果；滤波值越过180000 µg高阈值后产生事件并解除
 *       armed，持续高值不得重复计步。滤波值回落到126000 µg低阈值后
 *       才重新armed，随后在满足300 ms间隔时产生第二个事件。
 */
static void step_detector_test_b1(step_detector_test_stats_t *stats)
{
    step_detector_t detector;
    step_detector_input_t input;
    step_detector_output_t output;
    step_detector_process_result_t result;

    /* 使用任务10冻结的B1参数建立独立算法上下文。 */
    STEP_TEST_EXPECT(stats,
                    step_detector_init(
                        &detector,
                        &STEP_DETECTOR_CONFIG_B1_TASK10_V1) == 0,
                    "B1 init");

    /*
     * 第一份dynamic为0的样本直接初始化滤波状态和预热起点，
     * 不执行EWMA过渡，也不产生计步事件。
     */
    input = step_detector_test_make_dynamic_input(0U,1U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.filtered_dynamic_ug == 0U),
                    "B1 first sample initializes EWMA");

    /*
     * filtered = 0 +round((500000 - 0) × 100 / (120 + 100))= 227273 µg。
     *
     * 当前仅经过100 ms，仍在200 ms预热期内。
     */
    input = step_detector_test_make_dynamic_input(100U,2U,500000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.filtered_dynamic_ug == 227273U) &&
                    !output.event_generated,
                    "B1 first EWMA update and warmup");

    /*
     * 第二次EWMA更新：
     * 227273 + round((500000 - 227273) × 100 / 220)
     * = 351240 µg。
     *
     * 此时达到200 ms预热边界并越过高阈值，应产生首次事件，
     * 随后armed必须变为false。
     */
    input = step_detector_test_make_dynamic_input(200U,3U,500000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_EVENT) &&
                    (output.filtered_dynamic_ug == 351240U) &&
                    (output.total_step_count == 1U) &&
                    !detector.armed,
                    "B1 high threshold event disarms detector");

    /*
     * 持续输入高动态值后，滤波结果上升到418858 µg。
     * detector尚未重新armed，因此即使仍高于高阈值也不能重复计步。
     */
    input = step_detector_test_make_dynamic_input(300U,4U,500000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.filtered_dynamic_ug == 418858U) &&
                    !output.event_generated &&
                    !detector.armed,
                    "B1 high level cannot repeat while disarmed");

    /*
     * dynamic回落到0后，EWMA下降到228468 µg，
     * 但仍高于126000 µg低阈值，因此保持disarmed。
     */
    input = step_detector_test_make_dynamic_input(400U,5U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.filtered_dynamic_ug == 228468U) &&
                    !detector.armed,
                    "B1 remains disarmed above low threshold");

    /*
     * 下一份低动态样本使EWMA下降到124619 µg，
     * 已小于等于低阈值126000 µg，应重新进入armed状态。
     */
    input = step_detector_test_make_dynamic_input(500U,6U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.filtered_dynamic_ug == 124619U) &&
                    detector.armed,
                    "B1 rearms at low threshold");

    /*
     * 再次输入高动态值后，EWMA上升到295247 µg。
     * 距第一次事件已经400 ms，同时满足高阈值、armed和300 ms间隔，
     * 因此应产生第二个计步事件。
     */
    input = step_detector_test_make_dynamic_input(600U,7U,500000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_EVENT) &&
                    (output.filtered_dynamic_ug == 295247U) &&
                    (output.total_step_count == 2U),
                    "B1 produces second event after rearm and refractory");
}

/*
 * 简介：验证重复序号、正向缺样、长时间缺口、无效/过期样本和配置错误门禁。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：测试从一份合法样本开始，依次制造重复序号、正向序号跳变、
 *       101 ms长缺口、valid=false、stale=true和配置版本不匹配。
 *       被拒绝的样本不得进入特征或计步状态；故障解除后的首份合法新样本
 *       只能重新建立预热阶段，不能利用故障前状态立即产生事件。
 */
static void step_detector_test_input_contract(step_detector_test_stats_t *stats)
{
    step_detector_t detector;
    step_detector_input_t input;
    step_detector_output_t output;
    step_detector_process_result_t result;

    /* 使用冻结B1配置建立独立的输入契约测试上下文。 */
    STEP_TEST_EXPECT(stats,
                    step_detector_init(
                        &detector,
                        &STEP_DETECTOR_CONFIG_B1_TASK10_V1) == 0,
                    "input contract init");

    /*
     * tick=100、seq=10的首份合法样本只建立阶段状态。
     * 后续重复、缺样和时间差都以该样本为起点判断。
     */
    input = step_detector_test_make_dynamic_input(100U,10U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    result == STEP_DETECTOR_PROCESS_ACCEPTED,
                    "input contract initial sample");
    /*
     * 只修改tick并保留seq=10，构造同一器件样本被消费者重复读取的情况。
     * 重复样本不得更新previous_tick、滤波器、预热状态或累计步数。
     */
    input.tick = 120U;

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_IGNORED) &&
                    (output.reason == STEP_DETECTOR_REASON_DUPLICATE) &&
                    (detector.duplicate_count == 1U) &&
                    !output.sample_accepted,
                    "duplicate sequence ignored");

    /*
     * 序号从10前进到13，说明中间缺少11和12两份样本。
     * 当前seq=13样本仍是合法新样本，seq_gap_count应累计2。
     *
     * 重复样本没有更新previous_tick，因此本次有效时间差仍是
     * tick=200减去tick=100，正好100 ms，不应触发长缺口重置。
     */
    input = step_detector_test_make_dynamic_input(200U,13U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (detector.seq_gap_count == 2U),
                    "forward sequence gap counted");

    /*
     * tick从200前进到301，时间差为101 ms，严格大于冻结的
     * reset_gap_ms=100，因此必须清除滤波、预热和事件阶段。
     * 当前边界样本只建立新阶段，仍属于已接受样本，但不得计步。
     */
    input = step_detector_test_make_dynamic_input(301U,14U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    output.state_reset &&
                    (output.reason ==
                    STEP_DETECTOR_REASON_TIME_GAP_RESET) &&
                    (detector.reset_count == 1U),
                    "101 ms gap resets phase");

    /*
     * valid=false表示当前输入不是一份完整有效样本。
     * 由于301 ms样本已经重新建立活动阶段，本次无效输入应触发阶段重置。
     * 无效样本的seq=15不得成为previous_imu_seq。
     */
    input = step_detector_test_make_dynamic_input(321U,15U,0U);
    input.valid = false;

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_IGNORED) &&
                    output.state_reset &&
                    (output.reason == STEP_DETECTOR_REASON_INVALID) &&
                    (detector.invalid_count == 1U),
                    "invalid sample rejected and active phase reset");

    /*
     * stale=true表示这是过期的last-good值。
     * 即使数值本身看似正常，也不得再次用于滤波或计步。
     * 上一个invalid已经清除了活动阶段，因此本次不会重复统计同一次重置。
     */
    input = step_detector_test_make_dynamic_input(341U,15U,0U);
    input.stale = true;

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_IGNORED) &&
                    (output.reason == STEP_DETECTOR_REASON_STALE) &&
                    (detector.stale_count == 1U),
                    "stale last-good sample rejected");

    /*
     * 配置版本变化意味着量程、ODR或换算语义可能已经改变。
     * 算法不能把不同配置产生的数据继续放入同一个滤波和阈值状态。
     */
    input = step_detector_test_make_dynamic_input(361U,15U,0U);
    input.config_version =STEP_DETECTOR_TASK10_CONFIG_VERSION + 1U;

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_IGNORED) &&
                    (output.reason ==
                    STEP_DETECTOR_REASON_CONFIG_MISMATCH) &&
                    (detector.config_mismatch_count == 1U),
                    "configuration mismatch rejected");

    /*
     * 故障解除后重新提交seq=15的合法样本。
     * invalid、stale和配置不匹配路径均未接收该序号，因此它仍属于新样本。
     * 算法此时只能重新建立滤波和预热起点，不能立即计步。
     */
    input = step_detector_test_make_dynamic_input(381U,15U,0U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    (output.reason == STEP_DETECTOR_REASON_INITIALIZED) &&
                    !output.event_generated,
                    "valid sample restarts warmup after failure");
}


/*
 * 简介：验证32位系统tick和IMU样本序号的自然回绕及异常倒退处理。
 * 输入：stats为当前自测累计统计，由调用者持有并保证非NULL。
 * 返回值：无。
 * 说明：测试从UINT32_MAX附近开始，通过两个100 ms间隔跨越回绕点，
 *       保证每个相邻样本间隔都不超过reset_gap_ms，同时累计达到
 *       200 ms预热门限。随后制造序号倒退，验证阶段重置。
 */
static void step_detector_test_wraparound(step_detector_test_stats_t *stats)
{
    step_detector_t detector;
    step_detector_input_t input;
    step_detector_output_t output;
    step_detector_process_result_t result;
    rt_bool_t bridge_ok;

    STEP_TEST_EXPECT(stats,
                    step_detector_init(
                        &detector,
                        &STEP_DETECTOR_CONFIG_B0_TASK10_V1) == 0,
                    "wraparound init");

    /*
     * 起点位于UINT32_MAX之前100 tick。
     * 序号使用UINT32_MAX-1，为跨越序号回绕保留一个中间样本。
     */
    input = step_detector_test_make_dynamic_input(UINT32_MAX - 100U,UINT32_MAX - 1U,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    !output.event_generated,
                    "wraparound initial sample");

    /*
     * 第一段前进100 tick：
     *
     * tick：UINT32_MAX - (UINT32_MAX - 100) = 100
     * seq ：UINT32_MAX - (UINT32_MAX - 1) = 1
     *
     * 此时只达到100 ms预热，不能产生事件。
     */
    input = step_detector_test_make_dynamic_input(UINT32_MAX,UINT32_MAX,300000U);

    result = step_detector_process(&detector,&input,&output);

    bridge_ok =(result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                !output.event_generated &&
                !output.state_reset &&
                (detector.seq_gap_count == 0U) &&
                (detector.sequence_rollback_count == 0U);

    /*
     * 第二段跨越tick和序号回绕，再前进100 tick：
     *
     * tick：99 - UINT32_MAX（无符号）= 100
     * seq ：0 - UINT32_MAX（无符号）= 1
     *
     * 从预热起点累计经过200 ms，应在高阈值上产生事件。
     */
    input = step_detector_test_make_dynamic_input(99U,0U,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    bridge_ok &&
                    (result == STEP_DETECTOR_PROCESS_EVENT) &&
                    output.event_generated &&
                    (output.event_tick == 99U) &&
                    (detector.seq_gap_count == 0U) &&
                    (detector.sequence_rollback_count == 0U),
                    "tick and sequence natural wrap accepted");

    /*
     * 从序号0返回UINT32_MAX得到UINT32_MAX差值，
     * 应识别为异常倒退并重新建立预热阶段。
     */
    input = step_detector_test_make_dynamic_input(119U,UINT32_MAX,300000U);

    result = step_detector_process(&detector,&input,&output);

    STEP_TEST_EXPECT(stats,
                    (result == STEP_DETECTOR_PROCESS_ACCEPTED) &&
                    output.state_reset &&
                    !output.event_generated &&
                    (output.reason ==
                    STEP_DETECTOR_REASON_SEQUENCE_ROLLBACK_RESET) &&
                    (detector.sequence_rollback_count == 1U),
                    "sequence rollback resets phase");
}


/*
 * 简介：依次执行计步算法全部构造值测试，并输出最终通过与失败统计。
 * 输入：argc和argv由FinSH命令框架传入；当前命令不接收参数，二者仅为
 *       保持FinSH函数签名而保留。
 * 返回值：全部断言通过时返回RT_EOK；任一断言失败时返回-RT_ERROR。
 * 说明：测试覆盖配置门禁、整数模长、B0、B1、输入失败契约以及32位
 *       tick/序号回绕。命令只调用计步算法公开接口，不访问I²C、真实IMU、
 *       sensor服务或LVGL，也不执行延时和无限循环。所有用例执行完毕后
 *       统一输出SUMMARY，使一次UART日志可以完整保存所有失败位置。
 */
static int step_detector_self_test(int argc,char **argv)
{
    /* 统计结构位于FinSH线程栈中，每次执行命令都从全零状态开始。 */
    step_detector_test_stats_t stats = {0};

    /*
     * 当前命令不需要外部参数。
     * 显式转换为void，避免编译器产生未使用参数警告。
     */
    (void)argc;
    (void)argv;

    /*
     * BEGIN记录算法版本、配置版本和tick频率。
     * 后续保存UART证据时，可据此确认测试对应的运行语义。
     */
    rt_kprintf("[STEP][SELFTEST][BEGIN] "
                "algorithm_version=%u config_version=%u tick_hz=%u\n",
                (unsigned int)STEP_DETECTOR_ALGORITHM_VERSION,
                (unsigned int)STEP_DETECTOR_TASK10_CONFIG_VERSION,
                (unsigned int)STEP_DETECTOR_TASK10_TICK_HZ);

    /*
     * 每组测试使用自己的局部detector，彼此不共享算法运行状态。
     * 某项断言失败不会中止后续测试，以便一次收集完整诊断结果。
     */
    step_detector_test_guards(&stats);
    step_detector_test_magnitude(&stats);
    step_detector_test_b0(&stats);
    step_detector_test_b1(&stats);
    step_detector_test_input_contract(&stats);
    step_detector_test_wraparound(&stats);

    /*
     * 当前测试设计共有36项断言，但验收首先以fail=0和result=PASS
     * 为准。若以后增加合法测试项，应同步更新任务记录中的预期总数。
     */
    rt_kprintf("[STEP][SELFTEST][SUMMARY] "
                "total=%lu pass=%lu fail=%lu result=%s\n",
                (unsigned long)stats.total_count,
                (unsigned long)stats.pass_count,
                (unsigned long)stats.fail_count,
                (stats.fail_count == 0U) ? "PASS" : "FAIL");

    /*
     * 返回失败码可以让FinSH调用结果与SUMMARY保持一致，
     * 但完整失败原因仍以此前逐项输出的FAIL记录为准。
     */
    return (stats.fail_count == 0U)? RT_EOK: -RT_ERROR;
}

/*
 * 简介：比较一种目标板C算法产生的实际事件与Python生成的黄金事件。
 * 输入：algorithm为日志中使用的算法名称；actual为实际事件缓存；
 *       actual_count为实际事件总数；overflow_count为超出缓存容量的数量；
 *       expected为只读黄金事件数组；expected_count为黄金事件总数。
 * 返回值：返回事件数量、缓存溢出和逐字段比较产生的不匹配总数；
 *         返回0表示该算法的目标板事件与Python黄金事件完全一致。
 * 说明：函数先检查实际事件缓存溢出和事件总数，再比较双方共同存在且
 *       已保存的事件前缀。每个事件严格比较time_ms、dynamic_ug和
 *       filtered_dynamic_ug，不允许时间偏移、额外事件、遗漏事件或数值
 *       近似。函数只读取事件数组并输出日志，不修改任何算法上下文。
 */
static rt_uint32_t step_detector_golden_compare_events(const char *algorithm,
                                                        const step_detector_golden_event_t *actual,
                                                        rt_uint32_t actual_count,
                                                        rt_uint32_t overflow_count,
                                                        const step_detector_golden_event_t *expected,
                                                        rt_uint32_t expected_count)
{
    rt_uint32_t mismatch_count = 0U;
    rt_uint32_t compare_count;
    rt_uint32_t index;

    /*
     * actual数组容量有限。超过容量的事件虽然仍计入actual_count，
     * 但没有完整字段可供逐项比较，因此每个溢出事件直接记为不匹配。
     */
    if (overflow_count != 0U)
    {
        mismatch_count += overflow_count;

        rt_kprintf(
            "[STEP][GOLDEN][FAIL] algorithm=%s "
            "event_overflow=%lu\n",
            algorithm,
            (unsigned long)overflow_count);
    }

    /*
     * 事件总数不同本身就是一项失败。
     * 该检查可以发现Python事件被遗漏或目标板产生额外事件。
     */
    if (actual_count != expected_count)
    {
        mismatch_count++;

        rt_kprintf(
            "[STEP][GOLDEN][FAIL] algorithm=%s "
            "actual_events=%lu expected_events=%lu\n",
            algorithm,
            (unsigned long)actual_count,
            (unsigned long)expected_count);
    }

    /*
     * 逐字段比较只能覆盖双方共同存在的事件数量。
     * 同时不能超过actual数组实际保存容量，避免发生越界访问。
     */
    compare_count = actual_count;

    if (compare_count > expected_count)
    {
        compare_count = expected_count;
    }

    if (compare_count > STEP_GOLDEN_ACTUAL_EVENT_CAPACITY)
    {
        compare_count =STEP_GOLDEN_ACTUAL_EVENT_CAPACITY;
    }

    for (index = 0U;index < compare_count;index++)
    {
        /*
         * 黄金事件采用严格一致性比较：
         * 时间、未滤波dynamic和滤波结果三个字段都必须完全相同。
         */
        if ((actual[index].time_ms !=
             expected[index].time_ms) ||
            (actual[index].dynamic_ug !=
             expected[index].dynamic_ug) ||
            (actual[index].filtered_dynamic_ug !=
             expected[index].filtered_dynamic_ug))
        {
            mismatch_count++;

            /* 先输出事件编号，再分行输出实际值和预期值，避免日志过长。 */
            rt_kprintf(
                "[STEP][GOLDEN][FAIL] "
                "algorithm=%s event=%lu\n",
                algorithm,
                (unsigned long)(index + 1U));

            rt_kprintf(
                "[STEP][GOLDEN][ACTUAL] "
                "time=%lu dynamic=%lu filtered=%lu\n",
                (unsigned long)actual[index].time_ms,
                (unsigned long)actual[index].dynamic_ug,
                (unsigned long)
                    actual[index].filtered_dynamic_ug);

            rt_kprintf(
                "[STEP][GOLDEN][EXPECTED] "
                "time=%lu dynamic=%lu filtered=%lu\n",
                (unsigned long)expected[index].time_ms,
                (unsigned long)expected[index].dynamic_ug,
                (unsigned long)
                    expected[index].filtered_dynamic_ug);
        }
        else
        {
            /*
             * 匹配事件也保留完整字段，作为目标板确实执行了黄金窗口的
             * 正向证据，而不是只留下最终PASS。
             */
            rt_kprintf(
                "[STEP][GOLDEN][MATCH] "
                "algorithm=%s event=%lu "
                "time=%lu dynamic=%lu filtered=%lu\n",
                algorithm,
                (unsigned long)(index + 1U),
                (unsigned long)actual[index].time_ms,
                (unsigned long)actual[index].dynamic_ug,
                (unsigned long)
                    actual[index].filtered_dynamic_ug);
        }
    }

    /* 调用者以返回0作为该算法事件序列完全一致的判定条件。 */
    return mismatch_count;
}

/**
 * @brief 使用真实设计集黄金向量验证目标板C算法
 *
 * 本命令使用D_WALK_02的231份已冻结设计集样本，同时运行独立B0和B1
 * context。它不读取I2C、不访问sensor服务、不修改UI，也不重新运行
 * validation。所有运算均为有限循环。
 */
static int step_detector_golden_test(
    int argc,
    char **argv)
{
    step_detector_t b0_detector;
    step_detector_t b1_detector;

    step_detector_input_t input;
    step_detector_output_t b0_output;
    step_detector_output_t b1_output;

    step_detector_process_result_t b0_result;
    step_detector_process_result_t b1_result;

    step_detector_golden_event_t b0_actual_events[STEP_GOLDEN_ACTUAL_EVENT_CAPACITY] = {0};

    step_detector_golden_event_t b1_actual_events[STEP_GOLDEN_ACTUAL_EVENT_CAPACITY] = {0};

    step_detector_golden_stats_t stats = {0};

    rt_size_t compiled_sample_count;
    rt_size_t index;
    rt_bool_t process_ok;
    rt_bool_t feature_ok;
    rt_bool_t final_state_ok;
    rt_bool_t overall_pass;

    (void)argc;
    (void)argv;

    compiled_sample_count =sizeof(g_step_detector_golden_samples) /sizeof(g_step_detector_golden_samples[0]);

    rt_kprintf(
        "[STEP][GOLDEN][BEGIN] "
        "tool=%s segment=%s start=%u end=%u\n",
        STEP_GOLDEN_TOOL_VERSION,
        STEP_GOLDEN_SEGMENT_ID,
        (unsigned int)STEP_GOLDEN_START_MS,
        (unsigned int)STEP_GOLDEN_END_MS);

    rt_kprintf(
        "[STEP][GOLDEN][SOURCE] samples_sha256=%s\n",
        STEP_GOLDEN_SAMPLES_SHA256);

    rt_kprintf(
        "[STEP][GOLDEN][SOURCE] events_sha256=%s\n",
        STEP_GOLDEN_EVENTS_SHA256);

    /*
     * 宏值和实际数组长度不一致说明生成文件被手工改坏，
     * 此时禁止继续运行，以免越界或产生不可解释的证据。
     */
    if (compiled_sample_count !=STEP_GOLDEN_SAMPLE_COUNT)
    {
        rt_kprintf(
            "[STEP][GOLDEN][FAIL] "
            "compiled_samples=%lu declared_samples=%u\n",
            (unsigned long)compiled_sample_count,
            (unsigned int)STEP_GOLDEN_SAMPLE_COUNT);

        return -RT_ERROR;
    }

    if (step_detector_init(&b0_detector,&STEP_DETECTOR_CONFIG_B0_TASK10_V1) != 0)
    {
        rt_kprintf("[STEP][GOLDEN][FAIL] B0 init failed\n");

        return -RT_ERROR;
    }

    if (step_detector_init(&b1_detector, &STEP_DETECTOR_CONFIG_B1_TASK10_V1) != 0)
    {
        rt_kprintf("[STEP][GOLDEN][FAIL] B1 init failed\n");

        return -RT_ERROR;
    }

    for (index = 0U;index < compiled_sample_count; index++)
    {
        const step_detector_golden_sample_t *sample = &g_step_detector_golden_samples[index];

        /*
         * 使用数据集中的真实sample_tick和imu_seq。
         * time_ms只用于与Python事件时间进行对照。
         */
        input.tick = sample->sample_tick;
        input.imu_seq = sample->imu_seq;

        input.ax_ug = sample->ax_ug;
        input.ay_ug = sample->ay_ug;
        input.az_ug = sample->az_ug;

        input.config_version =STEP_DETECTOR_TASK10_CONFIG_VERSION;

        input.valid = true;
        input.stale = false;

        b0_result = step_detector_process(
            &b0_detector,
            &input,
            &b0_output);

        b1_result = step_detector_process(
            &b1_detector,
            &input,
            &b1_output);

        stats.sample_count++;

        /*
         * 该窗口不存在无效、过期、重复、缺口或配置变化。
         * 每个输入必须被两种算法接受，事件返回值必须与event标志一致。
         */
        process_ok =
            ((b0_result ==
              STEP_DETECTOR_PROCESS_ACCEPTED) ||
             (b0_result ==
              STEP_DETECTOR_PROCESS_EVENT)) &&
            b0_output.sample_accepted &&
            ((b0_result ==
              STEP_DETECTOR_PROCESS_EVENT) ==
             b0_output.event_generated) &&
            ((b1_result ==
              STEP_DETECTOR_PROCESS_ACCEPTED) ||
             (b1_result ==
              STEP_DETECTOR_PROCESS_EVENT)) &&
            b1_output.sample_accepted &&
            ((b1_result ==
              STEP_DETECTOR_PROCESS_EVENT) ==
             b1_output.event_generated);

        if (!process_ok)
        {
            stats.process_mismatch_count++;

            if (stats.process_mismatch_count <= 8U)
            {
                rt_kprintf(
                    "[STEP][GOLDEN][FAIL] "
                    "process index=%lu time=%lu "
                    "b0_result=%d b1_result=%d\n",
                    (unsigned long)index,
                    (unsigned long)sample->time_ms,
                    (int)b0_result,
                    (int)b1_result);
            }
        }

        /*
         * Python使用math.isqrt，C使用整数平方根。
         * 两种算法共享相同特征，四个值都必须逐样本完全一致。
         */
        feature_ok =
            (b0_output.magnitude_ug ==
             sample->expected_magnitude_ug) &&
            (b0_output.dynamic_ug ==
             sample->expected_dynamic_ug) &&
            (b1_output.magnitude_ug ==
             sample->expected_magnitude_ug) &&
            (b1_output.dynamic_ug ==
             sample->expected_dynamic_ug);

        if (!feature_ok)
        {
            stats.feature_mismatch_count++;

            if (stats.feature_mismatch_count <= 8U)
            {
                rt_kprintf(
                    "[STEP][GOLDEN][FAIL] "
                    "feature index=%lu time=%lu\n",
                    (unsigned long)index,
                    (unsigned long)sample->time_ms);

                rt_kprintf(
                    "[STEP][GOLDEN][ACTUAL] "
                    "b0_mag=%lu b0_dyn=%lu "
                    "b1_mag=%lu b1_dyn=%lu\n",
                    (unsigned long)
                        b0_output.magnitude_ug,
                    (unsigned long)
                        b0_output.dynamic_ug,
                    (unsigned long)
                        b1_output.magnitude_ug,
                    (unsigned long)
                        b1_output.dynamic_ug);

                rt_kprintf(
                    "[STEP][GOLDEN][EXPECTED] "
                    "mag=%lu dynamic=%lu\n",
                    (unsigned long)
                        sample->expected_magnitude_ug,
                    (unsigned long)
                        sample->expected_dynamic_ug);
            }
        }

        if (b0_result ==STEP_DETECTOR_PROCESS_EVENT)
        {
            rt_uint32_t event_index =
                stats.b0_actual_event_count;

            if (event_index <
                STEP_GOLDEN_ACTUAL_EVENT_CAPACITY)
            {
                b0_actual_events[event_index].time_ms =
                    sample->time_ms;

                b0_actual_events[event_index].dynamic_ug =
                    b0_output.dynamic_ug;

                b0_actual_events[event_index]
                    .filtered_dynamic_ug =
                    b0_output.filtered_dynamic_ug;
            }
            else
            {
                stats.b0_event_overflow_count++;
            }

            stats.b0_actual_event_count++;
        }

        if (b1_result ==STEP_DETECTOR_PROCESS_EVENT)
        {
            rt_uint32_t event_index =
                stats.b1_actual_event_count;

            if (event_index <
                STEP_GOLDEN_ACTUAL_EVENT_CAPACITY)
            {
                b1_actual_events[event_index].time_ms =
                    sample->time_ms;

                b1_actual_events[event_index].dynamic_ug =
                    b1_output.dynamic_ug;

                b1_actual_events[event_index]
                    .filtered_dynamic_ug =
                    b1_output.filtered_dynamic_ug;
            }
            else
            {
                stats.b1_event_overflow_count++;
            }

            stats.b1_actual_event_count++;
        }
    }

    stats.b0_event_mismatch_count =
        step_detector_golden_compare_events(
            "B0",
            b0_actual_events,
            stats.b0_actual_event_count,
            stats.b0_event_overflow_count,
            g_step_detector_golden_b0_events,
            STEP_GOLDEN_B0_EVENT_COUNT);

    stats.b1_event_mismatch_count =
        step_detector_golden_compare_events(
            "B1",
            b1_actual_events,
            stats.b1_actual_event_count,
            stats.b1_event_overflow_count,
            g_step_detector_golden_b1_events,
            STEP_GOLDEN_B1_EVENT_COUNT);

    /*
     * 连续黄金窗口不应出现重复、缺样、重置或失败门禁。
     * 最终累计步数必须等于实际记录到的事件数量。
     */
    final_state_ok =
        (b0_detector.accepted_count ==
         STEP_GOLDEN_SAMPLE_COUNT) &&
        (b1_detector.accepted_count ==
         STEP_GOLDEN_SAMPLE_COUNT) &&

        (b0_detector.step_count ==
         stats.b0_actual_event_count) &&
        (b1_detector.step_count ==
         stats.b1_actual_event_count) &&

        (b0_detector.duplicate_count == 0U) &&
        (b1_detector.duplicate_count == 0U) &&
        (b0_detector.seq_gap_count == 0U) &&
        (b1_detector.seq_gap_count == 0U) &&
        (b0_detector.reset_count == 0U) &&
        (b1_detector.reset_count == 0U) &&

        (b0_detector.invalid_count == 0U) &&
        (b1_detector.invalid_count == 0U) &&
        (b0_detector.stale_count == 0U) &&
        (b1_detector.stale_count == 0U) &&

        (b0_detector.config_mismatch_count == 0U) &&
        (b1_detector.config_mismatch_count == 0U) &&
        (b0_detector.sequence_rollback_count == 0U) &&
        (b1_detector.sequence_rollback_count == 0U);

    if (!final_state_ok)
    {
        stats.final_state_mismatch_count++;

        rt_kprintf(
            "[STEP][GOLDEN][FAIL] final state mismatch\n");

        rt_kprintf(
            "[STEP][GOLDEN][STATE] "
            "B0 accepted=%lu steps=%lu gap=%lu reset=%lu\n",
            (unsigned long)b0_detector.accepted_count,
            (unsigned long)b0_detector.step_count,
            (unsigned long)b0_detector.seq_gap_count,
            (unsigned long)b0_detector.reset_count);

        rt_kprintf(
            "[STEP][GOLDEN][STATE] "
            "B1 accepted=%lu steps=%lu gap=%lu reset=%lu\n",
            (unsigned long)b1_detector.accepted_count,
            (unsigned long)b1_detector.step_count,
            (unsigned long)b1_detector.seq_gap_count,
            (unsigned long)b1_detector.reset_count);
    }

    overall_pass =
        (stats.sample_count ==
         STEP_GOLDEN_SAMPLE_COUNT) &&
        (stats.process_mismatch_count == 0U) &&
        (stats.feature_mismatch_count == 0U) &&
        (stats.b0_event_mismatch_count == 0U) &&
        (stats.b1_event_mismatch_count == 0U) &&
        (stats.final_state_mismatch_count == 0U);

    /*
     * 分行输出，避免单行超过当前控制台缓冲能力。
     */
    rt_kprintf(
        "[STEP][GOLDEN][SUMMARY] "
        "samples=%lu process_mismatch=%lu "
        "feature_mismatch=%lu\n",
        (unsigned long)stats.sample_count,
        (unsigned long)
            stats.process_mismatch_count,
        (unsigned long)
            stats.feature_mismatch_count);

    rt_kprintf(
        "[STEP][GOLDEN][SUMMARY] "
        "B0 actual=%lu expected=%u mismatch=%lu\n",
        (unsigned long)
            stats.b0_actual_event_count,
        (unsigned int)
            STEP_GOLDEN_B0_EVENT_COUNT,
        (unsigned long)
            stats.b0_event_mismatch_count);

    rt_kprintf(
        "[STEP][GOLDEN][SUMMARY] "
        "B1 actual=%lu expected=%u mismatch=%lu\n",
        (unsigned long)
            stats.b1_actual_event_count,
        (unsigned int)
            STEP_GOLDEN_B1_EVENT_COUNT,
        (unsigned long)
            stats.b1_event_mismatch_count);

    rt_kprintf(
        "[STEP][GOLDEN][RESULT] %s\n",
        overall_pass ? "PASS" : "FAIL");

    return overall_pass
        ? RT_EOK
        : -RT_ERROR;
}
/*
 * 简介：计算在线算法输出步数与人工参考步数之间的绝对误差。
 * 输入：actual为算法输出步数；expected为人工参考步数。
 * 返回值：两者的无符号绝对差。
 * 说明：使用大小比较避免无符号减法下溢，不计算相对误差。
 *       静止和非步行场景的expected为0，不能计算相对误差。
 */
static rt_uint32_t step_detector_online_abs_diff(
                                    rt_uint32_t actual,
                                    rt_uint32_t expected)
{
    return (actual >= expected)
        ? (actual - expected)
        : (expected - actual);
}

/*
 * 简介：在有限时间内读取统一sensor服务快照，同时运行冻结的B0和B1算法。
 * 输入：argc/argv来自FinSH。
 *       argv[1]可选，表示测试时长，范围5～180秒，默认30秒。
 *       argv[2]可选，表示本次动作的人工参考步数。
 * 返回值：执行链完整返回RT_EOK；参数、服务、快照或算法处理异常返回错误。
 * 说明：
 * 1. 本命令只复制sensor服务快照，不访问I2C和器件寄存器；
 * 2. 不修改sensor线程周期、故障注入、UI对象或显示状态；
 * 3. B0和B1使用彼此独立的context；
 * 4. 每10 ms读取一次快照，重复IMU序号由算法契约拒绝；
 * 5. 只打印事件和最终统计，不逐样本打印，避免UART影响调度；
 * 6. RESULT=PASS只表示在线执行链完整，算法精度单独根据人工参考评价。
 */
static int step_detector_online_test(int argc,char **argv)
{
    step_detector_t b0_detector;
    step_detector_t b1_detector;

    step_detector_input_t input;
    step_detector_output_t b0_output;
    step_detector_output_t b1_output;

    step_detector_process_result_t b0_result;
    step_detector_process_result_t b1_result;

    step_detector_online_stats_t online_stats;
    sensor_snapshot_t snapshot;
    sensor_service_stats_t service_before;
    sensor_service_stats_t service_after;

    rt_uint32_t duration_s =STEP_ONLINE_DEFAULT_DURATION_S;
    rt_uint32_t reference_steps =0U;
    rt_bool_t has_reference =RT_FALSE;

    rt_bool_t has_last_seq =RT_FALSE;
    rt_uint32_t last_imu_seq =0U;

    rt_tick_t start_tick;
    rt_tick_t now_tick;
    rt_tick_t duration_ticks;
    rt_tick_t process_start;
    rt_tick_t process_ticks;

    rt_err_t result;
    rt_bool_t execution_pass;
    char *end_pointer;

    rt_memset(&online_stats,0,sizeof(online_stats));
    rt_memset(&snapshot,0,sizeof(snapshot));
    rt_memset(&service_before,0,sizeof(service_before));
    rt_memset(&service_after,0,sizeof(service_after));

    if ((argc < 1) || (argc > 3))
    {
        rt_kprintf(
            "usage: step_detector_online_test "
            "[duration_s 5..180] [reference_steps 0..1000]\n");
        return -RT_EINVAL;
    }

    if (argc >= 2)
    {
        unsigned long value;

        if (argv[1][0] == '-')
        {
            rt_kprintf("[STEP][ONLINE][FAIL] invalid duration\n");
            return -RT_EINVAL;
        }

        end_pointer =RT_NULL;
        value =strtoul(argv[1],&end_pointer,10);

        if ((end_pointer == argv[1]) ||
            (*end_pointer != '\0') ||
            (value < STEP_ONLINE_MIN_DURATION_S) ||
            (value > STEP_ONLINE_MAX_DURATION_S))
        {
            rt_kprintf(
                "[STEP][ONLINE][FAIL] duration must be 5..180 seconds\n");
            return -RT_EINVAL;
        }

        duration_s =(rt_uint32_t)value;
    }

    if (argc >= 3)
    {
        unsigned long value;

        if (argv[2][0] == '-')
        {
            rt_kprintf("[STEP][ONLINE][FAIL] invalid reference steps\n");
            return -RT_EINVAL;
        }

        end_pointer =RT_NULL;
        value =strtoul(argv[2],&end_pointer,10);

        if ((end_pointer == argv[2]) ||
            (*end_pointer != '\0') ||
            (value > STEP_ONLINE_MAX_REFERENCE_STEPS))
        {
            rt_kprintf(
                "[STEP][ONLINE][FAIL] reference steps must be 0..1000\n");
            return -RT_EINVAL;
        }

        reference_steps =(rt_uint32_t)value;
        has_reference =RT_TRUE;
    }

    /*
     * 诊断命令不负责隐式启动sensor服务。
     * 服务未运行表示前置条件不成立，应直接报告而不是改变系统生命周期。
     */
    if (sensor_service_is_running() == RT_FALSE)
    {
        rt_kprintf("[STEP][ONLINE][FAIL] sensor service is not running\n");
        return -RT_ENOSYS;
    }

    result =sensor_service_get_stats(&service_before);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[STEP][ONLINE][FAIL] get start stats result=%d\n",
            (int)result);
        return result;
    }

    if ((step_detector_init(&b0_detector,&STEP_DETECTOR_CONFIG_B0_TASK10_V1) != 0) ||
        (step_detector_init(&b1_detector,&STEP_DETECTOR_CONFIG_B1_TASK10_V1) != 0))
    {
        rt_kprintf("[STEP][ONLINE][FAIL] detector init failed\n");
        return -RT_ERROR;
    }

    duration_ticks =rt_tick_from_millisecond(duration_s * 1000U);
    if (duration_ticks == 0U)
    {
        duration_ticks =1U;
    }

    start_tick =rt_tick_get();

    rt_kprintf(
        "[STEP][ONLINE][BEGIN] duration_s=%lu poll_ms=%u "
        "algorithm_version=%u config_version=%u\n",
        (unsigned long)duration_s,
        (unsigned int)STEP_ONLINE_POLL_MS,
        (unsigned int)STEP_DETECTOR_ALGORITHM_VERSION,
        (unsigned int)STEP_DETECTOR_TASK10_CONFIG_VERSION);

    if (has_reference)
    {
        rt_kprintf(
            "[STEP][ONLINE][REFERENCE] steps=%lu\n",
            (unsigned long)reference_steps);
    }
    else
    {
        rt_kprintf("[STEP][ONLINE][REFERENCE] steps=NA\n");
    }

    while ((rt_tick_t)(rt_tick_get() - start_tick) < duration_ticks)
    {
        rt_bool_t contract_ok =RT_TRUE;

        online_stats.poll_count++;

        result =sensor_service_get_snapshot(&snapshot);
        if (result != RT_EOK)
        {
            online_stats.snapshot_error_count++;
            rt_thread_mdelay(STEP_ONLINE_POLL_MS);
            continue;
        }

        online_stats.snapshot_success_count++;

        /*
         * status与last-good样本来自同一份一致快照。
         * 二者的配置版本和采样tick不一致说明快照契约被破坏，
         * 该数据不得继续作为正常算法输入。
         */
        if (snapshot.imu_status.valid)
        {
            if ((snapshot.imu_status.config_version !=
                 snapshot.imu.config_version) ||
                (snapshot.imu_status.sample_tick !=
                 snapshot.imu.sample_tick))
            {
                contract_ok =RT_FALSE;
                online_stats.snapshot_contract_mismatch_count++;
            }
        }

        if (!has_last_seq ||
            (snapshot.imu_status.seq != last_imu_seq))
        {
            online_stats.new_imu_seq_count++;
            last_imu_seq =snapshot.imu_status.seq;
            has_last_seq =RT_TRUE;
        }
        else
        {
            online_stats.duplicate_snapshot_count++;
        }

        /*
         * 使用器件样本序号，不使用服务级publish_seq。
         * publish_seq可能由MMC或ALS更新，不能代表产生了新IMU样本。
         */
        input.tick =(rt_uint32_t)snapshot.imu.sample_tick;
        input.imu_seq =snapshot.imu_status.seq;

        input.ax_ug =snapshot.imu.accel_ug.x;
        input.ay_ug =snapshot.imu.accel_ug.y;
        input.az_ug =snapshot.imu.accel_ug.z;

        input.config_version =snapshot.imu.config_version;

        input.valid =
            (snapshot.imu_status.valid != RT_FALSE) &&
            (snapshot.imu.valid != RT_FALSE) &&
            (contract_ok != RT_FALSE);

        input.stale =
            (snapshot.imu_status.stale != RT_FALSE);

        if (!input.valid)
        {
            online_stats.invalid_input_count++;
        }
        else if (input.stale)
        {
            online_stats.stale_input_count++;
        }
        else
        {
            online_stats.valid_input_count++;
        }

        /*
         * 计时只覆盖两次纯算法调用。
         * 快照复制、UART输出和10 ms等待不计入算法处理耗时。
         */
        process_start =rt_tick_get();

        b0_result =step_detector_process(
            &b0_detector,
            &input,
            &b0_output);

        b1_result =step_detector_process(
            &b1_detector,
            &input,
            &b1_output);

        process_ticks =(rt_tick_t)(rt_tick_get() - process_start);

        if (process_ticks > online_stats.max_process_ticks)
        {
            online_stats.max_process_ticks =process_ticks;
        }

        if ((b0_result == STEP_DETECTOR_PROCESS_ERROR) ||
            (b1_result == STEP_DETECTOR_PROCESS_ERROR))
        {
            online_stats.process_error_count++;
        }

        if (b0_result == STEP_DETECTOR_PROCESS_EVENT)
        {
            rt_kprintf(
                "[STEP][ONLINE][EVENT] algorithm=B0 "
                "count=%lu tick=%lu dynamic=%lu filtered=%lu\n",
                (unsigned long)b0_output.total_step_count,
                (unsigned long)b0_output.event_tick,
                (unsigned long)b0_output.dynamic_ug,
                (unsigned long)b0_output.filtered_dynamic_ug);
        }

        if (b1_result == STEP_DETECTOR_PROCESS_EVENT)
        {
            rt_kprintf(
                "[STEP][ONLINE][EVENT] algorithm=B1 "
                "count=%lu tick=%lu dynamic=%lu filtered=%lu\n",
                (unsigned long)b1_output.total_step_count,
                (unsigned long)b1_output.event_tick,
                (unsigned long)b1_output.dynamic_ug,
                (unsigned long)b1_output.filtered_dynamic_ug);
        }

        rt_thread_mdelay(STEP_ONLINE_POLL_MS);
    }

    now_tick =rt_tick_get();

    result =sensor_service_get_stats(&service_after);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[STEP][ONLINE][FAIL] get end stats result=%d\n",
            (int)result);
        return result;
    }

    execution_pass =
        (online_stats.snapshot_success_count > 0U) &&
        (online_stats.new_imu_seq_count > 0U) &&
        (online_stats.snapshot_error_count == 0U) &&
        (online_stats.snapshot_contract_mismatch_count == 0U) &&
        (online_stats.process_error_count == 0U) &&
        (b0_detector.config_mismatch_count == 0U) &&
        (b1_detector.config_mismatch_count == 0U) &&
        (b0_detector.sequence_rollback_count == 0U) &&
        (b1_detector.sequence_rollback_count == 0U) &&
        (sensor_service_is_running() != RT_FALSE);

    rt_kprintf(
        "[STEP][ONLINE][SUMMARY] elapsed_ticks=%lu "
        "poll=%lu snapshot_ok=%lu snapshot_error=%lu\n",
        (unsigned long)(now_tick - start_tick),
        (unsigned long)online_stats.poll_count,
        (unsigned long)online_stats.snapshot_success_count,
        (unsigned long)online_stats.snapshot_error_count);

    rt_kprintf(
        "[STEP][ONLINE][SUMMARY] new_imu=%lu duplicate=%lu "
        "valid=%lu invalid=%lu stale=%lu contract_mismatch=%lu\n",
        (unsigned long)online_stats.new_imu_seq_count,
        (unsigned long)online_stats.duplicate_snapshot_count,
        (unsigned long)online_stats.valid_input_count,
        (unsigned long)online_stats.invalid_input_count,
        (unsigned long)online_stats.stale_input_count,
        (unsigned long)
            online_stats.snapshot_contract_mismatch_count);

    rt_kprintf(
        "[STEP][ONLINE][SUMMARY] B0_steps=%lu B1_steps=%lu "
        "process_error=%lu max_process_ticks=%lu\n",
        (unsigned long)b0_detector.step_count,
        (unsigned long)b1_detector.step_count,
        (unsigned long)online_stats.process_error_count,
        (unsigned long)online_stats.max_process_ticks);

    rt_kprintf(
        "[STEP][ONLINE][SERVICE] imu_success_delta=%lu "
        "imu_failure_delta=%lu imu_missed_delta=%lu\n",
        (unsigned long)
            (service_after.imu.success_count -
             service_before.imu.success_count),
        (unsigned long)
            (service_after.imu.failure_count -
             service_before.imu.failure_count),
        (unsigned long)
            (service_after.imu.missed_count -
             service_before.imu.missed_count));

    rt_kprintf(
        "[STEP][ONLINE][ALGORITHM] "
        "B0 accepted=%lu duplicate=%lu gap=%lu reset=%lu "
        "B1 accepted=%lu duplicate=%lu gap=%lu reset=%lu\n",
        (unsigned long)b0_detector.accepted_count,
        (unsigned long)b0_detector.duplicate_count,
        (unsigned long)b0_detector.seq_gap_count,
        (unsigned long)b0_detector.reset_count,
        (unsigned long)b1_detector.accepted_count,
        (unsigned long)b1_detector.duplicate_count,
        (unsigned long)b1_detector.seq_gap_count,
        (unsigned long)b1_detector.reset_count);

    if (has_reference)
    {
        rt_kprintf(
            "[STEP][ONLINE][ACCURACY] reference=%lu "
            "B0=%lu B0_abs_error=%lu "
            "B1=%lu B1_abs_error=%lu\n",
            (unsigned long)reference_steps,
            (unsigned long)b0_detector.step_count,
            (unsigned long)step_detector_online_abs_diff(
                b0_detector.step_count,
                reference_steps),
            (unsigned long)b1_detector.step_count,
            (unsigned long)step_detector_online_abs_diff(
                b1_detector.step_count,
                reference_steps));
    }

    rt_kprintf(
        "[STEP][ONLINE][RESULT] %s\n",
        execution_pass ? "PASS" : "FAIL");

    return execution_pass ? RT_EOK : -RT_ERROR;
}

MSH_CMD_EXPORT(step_detector_self_test,run deterministic step detector self tests);
MSH_CMD_EXPORT(step_detector_golden_test,compare target C algorithm with Python golden vectors);
MSH_CMD_EXPORT(step_detector_online_test,run bounded online B0 and B1 comparison);
#endif /* RT_USING_FINSH */