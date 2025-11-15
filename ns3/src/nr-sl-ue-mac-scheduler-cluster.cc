#include "nr-sl-ue-mac-scheduler-cluster.h"

#include "nr-sl-ue-mac-harq.h"
#include "nr-ue-mac.h"

#include <ns3/boolean.h>
#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/uinteger.h>

#include <optional>
#include <queue>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrSlUeMacSchedulerCluster");
NS_OBJECT_ENSURE_REGISTERED(NrSlUeMacSchedulerCluster);
// 注册调度器TypeID
TypeId NrSlUeMacSchedulerCluster::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::NrSlUeMacSchedulerCluster")
        .SetParent<NrSlUeMacSchedulerFixedMcs>()
        .AddConstructor<NrSlUeMacSchedulerCluster>()
        .SetGroupName("nr")
        // 可添加自定义属性（如配置RB起始索引和数量）
        // .AddAttribute("Mcs",
        //                 "The fixed value of the MCS used by this scheduler",
        //                 UintegerValue(14),
        //                 MakeUintegerAccessor(&NrSlUeMacSchedulerCluster::m_mcs),
        //                 MakeUintegerChecker<uint8_t>())
        // .AddAttribute("PriorityToSps",
        //                 "Flag to give scheduling priority to logical channels that are "
        //                 "configured with SPS in case of priority tie",
        //                 BooleanValue(true),
        //                 MakeBooleanAccessor(&NrSlUeMacSchedulerCluster::m_prioToSps),
        //                 MakeBooleanChecker())
        // .AddAttribute("WholeSlotExclusion",
        //                 "Whether to exclude use of candidate resources when other resources "
        //                 "in same slot are sensed",
        //                 BooleanValue(false),
        //                 MakeBooleanAccessor(&NrSlUeMacSchedulerCluster::m_wholeSlotExclusion),
        //                 MakeBooleanChecker())
        // .AddAttribute("AllowMultipleDestinationsPerSlot",
        //                 "Allow scheduling of multiple destinations in same slot",
        //                 BooleanValue(false),
        //                 MakeBooleanAccessor(
        //                     &NrSlUeMacSchedulerCluster::m_allowMultipleDestinationsPerSlot),
        //                 MakeBooleanChecker())
        .AddAttribute("CustomRbStart", "Custom RB start index",
                      UintegerValue(5),
                      MakeUintegerAccessor(&NrSlUeMacSchedulerCluster::m_customRbStart),
                      MakeUintegerChecker<uint16_t>())
        .AddAttribute("CustomRbNum", "Custom RB number",
                      UintegerValue(10),
                      MakeUintegerAccessor(&NrSlUeMacSchedulerCluster::m_customRbNum),
                      MakeUintegerChecker<uint16_t>());
    return tid;
}

NrSlUeMacSchedulerCluster::NrSlUeMacSchedulerCluster()
{
    NS_LOG_FUNCTION(this);
    m_grantSelectionUniformVariable = CreateObject<UniformRandomVariable>();
    m_destinationUniformVariable = CreateObject<UniformRandomVariable>();
    m_ueSelectedUniformVariable = CreateObject<UniformRandomVariable>();
}


// std::list<SlResourceInfo> NrSlUeMacSchedulerCluster::SelectResourcesWithConstraint(std::list<SlResourceInfo> txOpps, bool harqEnabled)
// {
//     std::list<SlResourceInfo> selectedResources;
//     uint8_t maxTxNum = GetSlMaxTxTransNumPssch(); // 最大重传次数

//     // 遍历候选资源，筛选出符合自定义RB要求的资源
//     for (auto& res : txOpps)
//     {
//         // 自定义规则：只选择 起始RB = m_customRbStart 且 RB数量 = m_customRbNum 的资源
//         if (res.m_startSubCh == m_customRbStart && res.m_numSubCh == m_customRbNum)
//         {
//             selectedResources.push_back(res);
//             // 满足最大重传次数后停止筛选
//             if (harqEnabled && selectedResources.size() >= maxTxNum)
//                 break;
//             // 无HARQ时只选1个
//             if (!harqEnabled && selectedResources.size() >= 1)
//                 break;
//         }
//     }

//     // 若未找到符合要求的资源，降级使用默认逻辑（避免调度失败）
//     if (selectedResources.empty())
//     {
//         selectedResources = NrSlUeMacSchedulerCluster::SelectResourcesWithConstraint(txOpps, harqEnabled);
//     }

//     return selectedResources;
// }


// bool NrSlUeMacSchedulerCluster::DoNrSlAllocation(const std::list<SlResourceInfo>& candResources,
//                                         const std::shared_ptr<NrSlUeMacSchedulerDstInfo>& dstInfo,
//                                         std::set<SlGrantResource>& slotAllocList,
//                                         const AllocationInfo& allocationInfo)
// {
//     std::list<SlResourceInfo> selectedResources;

//     // 根据HARQ是否使能，调用对应的自定义资源选择函数
//     if (allocationInfo.m_harqEnabled)
//     {
//         selectedResources = SelectResourcesWithConstraint(candResources, true);
//     }
//     else
//     {
//         selectedResources = SelectResourcesForBlindRetransmissions(candResources);
//     }

//     // 将选中的资源（含自定义RB）转换为 SlGrantResource，填入 slotAllocList
//     for (auto& res : selectedResources)
//     {
//         SlGrantResource grantRes;
//         grantRes.m_sfnSf = res.m_sfnSf;       // 时隙信息
//         grantRes.m_startSubCh = res.m_startSubCh; // 自定义RB起始索引
//         grantRes.m_numSubCh = res.m_numSubCh;   // 自定义RB数量
//         grantRes.m_rv = 0; // 冗余版本（可按默认逻辑）
//         slotAllocList.insert(grantRes);
//     }

//     // 计算TBSize（复用父类逻辑，确保RB能承载数据）
//     uint32_t tbSize = CalculateTbSize(m_nrSlAmc,
//                                       m_mcs,
//                                       res.m_symbolsPerSlot,
//                                       m_customRbNum,
//                                       res.m_subChSize);

//     // 更新分配信息（TBSize、RLC PDU等），后续会生成grant下发
//     // （此处可复用父类对RLC PDU的分配逻辑，或自定义）
//     return !slotAllocList.empty(); // 分配成功返回true
// }

}