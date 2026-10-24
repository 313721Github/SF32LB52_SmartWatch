#ifndef STEP_DETECTOR_H
#define STEP_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STEP_DETECTOR_ALGORITHM_VERSION        1U       //计步算法实现版本；算法公式、状态机或边界语义变化时递增
#define STEP_DETECTOR_TASK10_CONFIG_VERSION    1U       //任务10冻结配置版本；阈值、时间常数或时间门限变化时递增
#define STEP_DETECTOR_TASK10_TICK_HZ           1000U    //任务10采集与在线对照使用的系统tick频率，单位：Hz

typedef enum
{
    STEP_DETECTOR_ALGORITHM_B0 = 0,      //固定dynamic阈值和最小事件间隔的基础算法
    STEP_DETECTOR_ALGORITHM_B1           //加入EWMA平滑、高低阈值滞回和最小事件间隔的改进算法
} step_detector_algorithm_t;

typedef enum
{
    STEP_DETECTOR_REASON_NONE = 0,              //当前处理没有产生额外状态原因
    STEP_DETECTOR_REASON_INITIALIZED,           //接收到第一份有效样本，已建立滤波、序号和预热起点
    STEP_DETECTOR_REASON_ACCEPTED,              //样本有效且为新样本，已参与特征、滤波和状态更新，但未产生事件
    STEP_DETECTOR_REASON_EVENT,                 //样本通过门限、滞回、预热和最小间隔检查，产生一次计步事件
    STEP_DETECTOR_REASON_DUPLICATE,             //imu_seq与上一份已处理样本相同，本次不更新滤波器且不计步
    STEP_DETECTOR_REASON_INVALID,               //输入样本无效，本次不计步并重置滤波、滞回和预热状态
    STEP_DETECTOR_REASON_STALE,                 //输入为过期的last-good样本，本次不计步并重置算法运行状态
    STEP_DETECTOR_REASON_CONFIG_MISMATCH,       //样本配置版本与算法预期版本不一致，需要完整重置并重新预热
    STEP_DETECTOR_REASON_TIME_GAP_RESET,        //相邻有效样本时间缺口超过上限，已重置状态且不在缺口边界计步
    STEP_DETECTOR_REASON_SEQUENCE_ROLLBACK_RESET//序号出现非正常倒退，已重置算法状态以避免继续使用错误时序
} step_detector_reason_t;

typedef enum
{
    STEP_DETECTOR_PROCESS_ERROR = -1,           //参数、配置或上下文非法，当前输入没有被处理
    STEP_DETECTOR_PROCESS_IGNORED = 0,          //输入因重复、无效、过期或触发重置而未参与正常计步
    STEP_DETECTOR_PROCESS_ACCEPTED = 1,         //新有效样本已被处理并更新算法状态，但没有产生计步事件
    STEP_DETECTOR_PROCESS_EVENT = 2             //新有效样本已被处理，并产生一次计步事件
} step_detector_process_result_t;

/**
 * @brief 计步算法冻结配置
 *
 * 所有时间参数都使用毫秒。tick_hz说明输入tick每秒的计数，
 * 算法使用真实tick差换算dt_ms，不依赖固定样本数量。
 */
typedef struct
{
    step_detector_algorithm_t algorithm;    //选择基础算法B0或改进算法B1

    uint32_t high_threshold_ug;             //计步触发高阈值，单位：µg；B0和B1均使用
    uint32_t low_threshold_ug;              //B1重新进入armed状态的低阈值，单位：µg；B0固定为0且不使用

    uint32_t refractory_ms;                 //两次计步事件之间允许的最小时间间隔，单位：ms
    uint32_t ewma_tau_ms;                   //B1的EWMA时间常数，单位：ms；B0固定为0且不使用
    uint32_t warmup_ms;                     //初始化或状态重置后的预热时间，单位：ms；预热期间更新状态但不计步
    uint32_t reset_gap_ms;                  //相邻有效样本超过该时间间隔时重置算法状态，单位：ms

    uint32_t tick_hz;                       //输入tick的频率，单位：Hz；用于把无符号tick差换算为毫秒
    uint32_t expected_config_version;       //允许参与计步的IMU配置版本；不匹配时重置算法并拒绝本次样本
} step_detector_config_t;

/**
 * @brief 一份纯算法输入
 *
 * 本结构不包含RT-Thread、I2C、驱动或LVGL对象。
 * 调用者负责把sensor快照转换为本结构。
 */
typedef struct
{
    uint32_t tick;              //IMU完整样本读取完成时的系统tick，对应sensor_imu_sample_t.sample_tick
    uint32_t imu_seq;           //IMU器件样本序号；用于识别重复、缺样、正常回绕和异常倒退

    int32_t ax_ug;              //X轴加速度，单位：µg
    int32_t ay_ug;              //Y轴加速度，单位：µg
    int32_t az_ug;              //Z轴加速度，单位：µg

    uint32_t config_version;    //产生该样本时使用的IMU配置版本

    bool valid;                 //输入是否来自一份完整有效样本；false时不得更新滤波器或计步
    bool stale;                 //输入是否已经过期；true时不得继续使用last-good值计步
} step_detector_input_t;

/**
 * @brief 单次处理结果
 *
 * 输出由调用者持有。算法不会返回内部缓冲区指针。
 */
typedef struct
{
    step_detector_reason_t reason;  //说明本次样本被接受、忽略、触发事件或触发重置的具体原因

    bool sample_accepted;           //本次输入是否作为一份新的有效样本参与了算法状态更新
    bool event_generated;           //本次处理是否产生了一次新的计步事件
    bool state_reset;               //本次处理是否因无效、过期、配置变化或时间/序号异常重置运行状态

    uint32_t magnitude_ug;          //三轴模长向下取整结果：floor(sqrt(ax²+ay²+az²))，单位：µg
    uint32_t dynamic_ug;            //去除1g后的绝对动态量：abs(magnitude_ug-1000000)，单位：µg
    uint32_t filtered_dynamic_ug;   //本次状态更新后的动态量；B0等于dynamic_ug，B1为EWMA结果

    uint32_t total_step_count;      //当前算法实例从初始化以来累计产生的计步事件数量
    uint32_t event_tick;            //本次事件对应的输入tick；event_generated=false时固定为0
} step_detector_output_t;

/**
 * @brief 单个算法实例的完整状态
 *
 * 在线同时比较B0和B1时，调用者分别创建两个context。
 * context不包含动态内存、锁、线程或设备句柄。
 */
typedef struct
{
    step_detector_config_t config;  //初始化时复制的完整配置，运行期间由本算法实例独占读取

    bool configured;                //context是否已经通过step_detector_init()取得合法配置
    bool initialized;               //滤波、时间和预热状态是否已经由第一份有效新样本建立
    bool armed;                     //B1是否允许下一次高阈值越界产生事件；B0保持true且不使用滞回
    bool has_previous_seq;          //previous_imu_seq是否已经包含可用于比较的样本序号
    bool has_last_event;            //last_event_tick是否已经包含一份有效计步事件时间

    uint32_t previous_tick;         //上一份已接受有效样本的tick，用于计算真实时间间隔和检测长缺口
    uint32_t previous_imu_seq;      //上一份已观察的新有效IMU样本序号，用于识别重复、缺样和倒退
    uint32_t warmup_start_tick;     //最近一次初始化或状态重置后的预热起点tick
    uint32_t last_event_tick;       //最近一次计步事件tick；只有has_last_event=true时才有效

    uint32_t filtered_dynamic_ug;   //当前滤波状态；B0保存最近dynamic，B1保存整数EWMA结果，单位：µg
    uint32_t step_count;            //当前算法实例累计输出的计步事件数量

    uint32_t accepted_count;        //被识别为新、有效且配置匹配并参与处理的样本总数
    uint32_t duplicate_count;       //因imu_seq与previous_imu_seq相同而被忽略的输入总数
    uint32_t seq_gap_count;         //序号向前跳变时累计缺失的样本数量，不包含当前已经收到的样本
    uint32_t reset_count;           //运行状态被重置的累计次数，不包含第一次正常初始化

    uint32_t invalid_count;         //因valid=false而被拒绝的输入总数
    uint32_t stale_count;           //因stale=true而被拒绝的输入总数
    uint32_t config_mismatch_count; //因输入配置版本与expected_config_version不一致而被拒绝的总数
    uint32_t sequence_rollback_count;//检测到非正常序号倒退并触发状态重置的累计次数
} step_detector_t;

/* 任务10已经冻结的B0、B1配置。 */
extern const step_detector_config_t STEP_DETECTOR_CONFIG_B0_TASK10_V1;  //任务10已经冻结的B0配置：固定阈值与500 ms最小事件间隔

extern const step_detector_config_t STEP_DETECTOR_CONFIG_B1_TASK10_V1;  //任务10已经冻结的B1配置：EWMA、双阈值滞回与300 ms最小事件间隔



int step_detector_init(step_detector_t *detector,const step_detector_config_t *config); //初始化一个算法实例初始化会复制配置并清零运行状态
void step_detector_reset(step_detector_t *detector);    //清除一个实例的全部运行数据,保留已验证的配置，清除步数、滤波状态和诊断计数
step_detector_process_result_t step_detector_process(step_detector_t *detector,
                                                    const step_detector_input_t *input,
                                                    step_detector_output_t *output);    //处理一份IMU输入,本函数只执行有限整数计算，不延时、不打印、不访问硬件

#ifdef __cplusplus
}
#endif

#endif