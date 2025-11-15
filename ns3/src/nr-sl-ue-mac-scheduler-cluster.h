#ifndef NR_SL_UE_MAC_SCHEDULER_CLUSTER_H
#define NR_SL_UE_MAC_SCHEDULER_CLUSTER_H

#include "nr-sl-phy-mac-common.h"
#include "nr-sl-ue-mac-harq.h"
#include "nr-sl-ue-mac-scheduler-dst-info.h"
#include "nr-sl-ue-mac-scheduler.h"
#include "nr-sl-ue-mac-scheduler-fixed-mcs.h"
#include "nr-sl-ue-mac.h"

#include <ns3/random-variable-stream.h>

#include <functional>
#include <list>
#include <memory>

namespace ns3
{

class NrSlUeMacSchedulerCluster : public NrSlUeMacSchedulerFixedMcs
{
public:
    static TypeId GetTypeId(void);
    NrSlUeMacSchedulerCluster();
    // ~NrSlUeMacSchedulerCluster() override;

protected:
    // 重写核心资源分配函数
    // bool DoNrSlAllocation(const std::list<SlResourceInfo>& candResources,
    //                       const std::shared_ptr<NrSlUeMacSchedulerDstInfo>& dstInfo,
    //                       std::set<SlGrantResource>& slotAllocList,
    //                       const AllocationInfo& allocationInfo) override;

    // // 重写资源选择函数（根据HARQ是否使能选择对应函数）
    // std::list<SlResourceInfo> SelectResourcesForBlindRetransmissions(std::list<SlResourceInfo> txOpps) override;
    // std::list<SlResourceInfo> SelectResourcesWithConstraint(std::list<SlResourceInfo> txOpps, bool harqEnabled) override;

private:
    // 自定义RB配置：例如指定使用RB起始索引5，长度10
    uint16_t m_customRbStart = 5;
    uint16_t m_customRbNum = 10;
};

} // namespace ns3

#endif