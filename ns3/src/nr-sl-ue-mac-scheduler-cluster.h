#ifndef NR_SL_UE_MAC_SCHEDULER_CLUSTER_H
#define NR_SL_UE_MAC_SCHEDULER_CLUSTER_H

#include "nr-sl-ue-mac-harq.h"
#include "nr-sl-ue-mac-scheduler-dst-info.h"
#include "nr-sl-ue-mac-scheduler.h"
#include "nr-sl-ue-mac-scheduler-fixed-mcs.h"
#include "nr-sl-ue-mac.h"

#include <ns3/random-variable-stream.h>
#include <mutex>
#include <vector>

namespace ns3
{

// 定义链路类型枚举
enum class LinkType {
    CLUSTER_INTERNAL,  // 簇内链路
    CLUSTER_EXTERNAL   // 簇间链路
};

// CARLA下发的传输指令结构体
struct CarlaTxCommand
{
    uint32_t srcL2Id;       // 发送方L2 ID
    uint32_t dstL2Id;       // 接收方L2 ID(广播为0)
    uint32_t dataSize;      // 数据大小(字节)
    uint8_t subchannel_start; // CARLA指定的子信道起始索引
    uint8_t subchannel_num;   // CARLA指定的子信道数量(通常1~4)
    double txPower;         // CARLA指定的发射功率(W, 可选)
    LinkType linkType;      // 链路类型(簇内/簇间)
};

class NrSlUeMacSchedulerCluster : public NrSlUeMacSchedulerFixedMcs
{
public:
    /**
     * rief GetTypeId
     * 
eturn The TypeId of the class
     */
    static TypeId GetTypeId(void);

    // CARLA调用此接口下发传输指令(核心接口)
    void AddCarlaTxCommand(const CarlaTxCommand& cmd);
    // 清空已执行的指令(避免重复调度)
    void ClearCompletedCommands();

private:
    // 重写调度触发函数, 优先执行CARLA指令
    void DoSchedNrSlTriggerReq(const SfnSf& sfn) override;

    // 执行CARLA指令, 创建NS-3的Grant并发送
    void ExecuteCarlaCommands(const SfnSf& sfn);

    // 为CARLA指令创建目标L2 ID的DstInfo
    void CreateDstInfoForCarlaCmd(uint32_t dstL2Id);

    // 存储CARLA下发的待执行指令(线程安全, 用互斥锁保护)
    std::vector<CarlaTxCommand> m_carlaTxCommands;
    std::mutex m_cmdMutex;
    
    // 配置参数
    uint8_t m_numChannels{10};      // 子信道数量
    double m_sinrThreshold{6.0};    // 冲突检测SINR阈值(dB)
    double m_maxTxPower{0.1};       // 最大发射功率(W)
    double m_noisePower{1e-17};     // 噪声功率(W)
};

} // namespace ns3

#endif