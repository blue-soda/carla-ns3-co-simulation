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
#include <queue>
#include <map>

namespace ns3
{
// CARLA下发的传输指令结构体
struct CarlaTxCommand {
    uint32_t srcL2Id;              // 源终端 L2 ID
    uint32_t dstL2Id;              // 目标终端 L2 ID
    uint8_t slSubchannelStart;     // 子信道起始位置
    uint8_t slSubchannelSize;      // 子信道数量
    int maxDataSize;               // 最大数据大小(字节)
    //以下参数未实装
    bool isDynamic = true;          // 是否为动态调度(true)或 SPS(false)
    uint8_t lcid;                   // 逻辑信道 ID
    uint8_t mcs;                    // MCS 值
    uint32_t tbSize;                // 传输块大小（字节）
    SfnSf sfn;                      // 期望发送的时隙（可选，用于同步）
    Time rri;                       // SPS 资源预留间隔（仅 SPS 时有效）
};

class NrSlUeMacSchedulerManual : public NrSlUeMacSchedulerFixedMcs
{
public:
    /**
     * GetTypeId
     * return The TypeId of the class
     */
    static TypeId GetTypeId(void);

    // CARLA调用此接口下发传输指令
    void AddCarlaTxCommand(const CarlaTxCommand& cmd);
    // 清空指令
    void ClearCompletedCommands();
    
private:
    // 重写逻辑信道优先级调度方法
    uint32_t LogicalChannelPrioritization(
        const SfnSf& sfn,
        std::map<uint32_t, std::vector<uint8_t>> dstsAndLcsToSched,
        AllocationInfo& allocationInfo,
        std::list<SlResourceInfo>& candResources) override;

    // 存储CARLA下发的待执行指令(线程安全, 用互斥锁保护)
    // std::queue<CarlaTxCommand> m_carlaTxCommands;
    std::map<uint32_t, std::queue<CarlaTxCommand>> m_carlaTxCommandsByDst;
    std::mutex m_cmdMutex;
};

} // namespace ns3

#endif