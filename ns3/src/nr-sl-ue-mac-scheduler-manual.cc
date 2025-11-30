#include "nr-sl-ue-mac-scheduler-manual.h"

#include "nr-sl-ue-mac-harq.h"
#include "nr-ue-mac.h"

#include <ns3/boolean.h>
#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/uinteger.h>
#include <ns3/double.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrSlUeMacSchedulerManual");
NS_OBJECT_ENSURE_REGISTERED(NrSlUeMacSchedulerManual);
// 注册调度器TypeID
TypeId
NrSlUeMacSchedulerManual::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::NrSlUeMacSchedulerManual")
        .SetParent<NrSlUeMacSchedulerFixedMcs>()
        .AddConstructor<NrSlUeMacSchedulerManual>();
    return tid;
}


// CARLA添加传输指令(线程安全)
void
NrSlUeMacSchedulerManual::AddCarlaTxCommand(const CarlaTxCommand& cmd)
{
    NS_LOG_FUNCTION(this << cmd.srcL2Id << cmd.dstL2Id << cmd.slSubchannelStart << cmd.slSubchannelSize);
    // std::cout << "[DEBUG] NrSlUeMacSchedulerManual::AddCarlaTxCommand called\n";
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_carlaTxCommandsByDst[cmd.dstL2Id].push(cmd);
}

// 清空指令
void
NrSlUeMacSchedulerManual::ClearCompletedCommands()
{
    NS_LOG_FUNCTION(this);
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_carlaTxCommandsByDst.clear();
}

// 重写逻辑信道优先级调度方法
uint32_t
NrSlUeMacSchedulerManual::LogicalChannelPrioritization(
    const SfnSf& sfn,
    std::map<uint32_t, std::vector<uint8_t>> dstsAndLcsToSched,
    AllocationInfo& allocationInfo,
    std::list<SlResourceInfo>& candResources)
{
    NS_LOG_FUNCTION(this << dstsAndLcsToSched.size() << candResources.size());

    if (dstsAndLcsToSched.size() == 0)
    {
        return 0;
    }
    m_reselCounter = 0;
    m_cResel = 0;

    // 1. Selection of destination and logical channels to allocate
    std::map<uint8_t, std::vector<uint32_t>> dstL2IdsbyPrio;
    for (auto& itDst : dstsAndLcsToSched)
    {
        uint8_t lcHighestPrio = 0;
        auto itDstInfo = m_dstMap.find(itDst.first);
        auto& lcgMap = itDstInfo->second->GetNrSlLCG();
        for (auto& itLc : itDst.second)
        {
            uint8_t lcPriority = lcgMap.begin()->second->GetLcPriority(itLc);
            NS_LOG_DEBUG("Destination L2 ID "
                         << itDst.first << " LCID " << +itLc << " priority " << +lcPriority
                         << " buffer size " << lcgMap.begin()->second->GetTotalSizeOfLC(itLc)
                         << " dynamic scheduling " << lcgMap.begin()->second->IsLcDynamic(itLc)
                         << " RRI " << (lcgMap.begin()->second->GetLcRri(itLc)).GetMilliSeconds()
                         << " ms");
            if (lcPriority > lcHighestPrio)
            {
                lcHighestPrio = lcPriority;
            }
        }
        auto itDstL2IdsbyPrio = dstL2IdsbyPrio.find(lcHighestPrio);
        if (itDstL2IdsbyPrio == dstL2IdsbyPrio.end())
        {
            std::vector<uint32_t> dstIds;
            dstIds.emplace_back(itDst.first);
            dstL2IdsbyPrio.emplace(lcHighestPrio, dstIds);
        }
        else
        {
            itDstL2IdsbyPrio->second.emplace_back(itDst.first);
        }
    }
    uint8_t dstHighestPrio = dstL2IdsbyPrio.rbegin()->first;
    NS_ASSERT_MSG(dstL2IdsbyPrio.rbegin()->second.size(), "Unexpected empty vector");
    uint32_t randomIndex =
        m_destinationUniformVariable->GetInteger(0, dstL2IdsbyPrio.rbegin()->second.size() - 1);
    uint32_t dstIdSelected = dstL2IdsbyPrio.rbegin()->second.at(randomIndex);
    NS_LOG_INFO("Selected dstL2ID "
                << dstIdSelected << " (" << dstL2IdsbyPrio.rbegin()->second.size() << "/"
                << dstsAndLcsToSched.size() << " destinations with highest LC priority of "
                << +dstHighestPrio << ")");

    // 1.2.Select destination's logical channels
    auto itDstInfo = m_dstMap.find(dstIdSelected);
    const auto& lcgMap = itDstInfo->second->GetNrSlLCG();
    const auto& itDst = dstsAndLcsToSched.find(dstIdSelected);
    std::map<uint8_t, std::vector<uint8_t>> lcIdsbyPrio;
    for (auto& itLc : itDst->second)
    {
        uint8_t lcPriority = lcgMap.begin()->second->GetLcPriority(itLc);
        auto itLcIdsbyPrio = lcIdsbyPrio.find(lcPriority);
        if (itLcIdsbyPrio == lcIdsbyPrio.end())
        {
            std::vector<uint8_t> lcIds;
            lcIds.emplace_back(itLc);
            lcIdsbyPrio.emplace(lcPriority, lcIds);
        }
        else
        {
            itLcIdsbyPrio->second.emplace_back(itLc);
        }
    }
    bool dynamicGrant = true;
    uint16_t nDynLcs = 0;
    uint16_t nSpsLcs = 0;
    if (lcIdsbyPrio.rbegin()->second.size() > 1)
    {
        for (auto& itLcsHighestPrio : lcIdsbyPrio.rbegin()->second)
        {
            if (lcgMap.begin()->second->IsLcDynamic(itLcsHighestPrio))
            {
                nDynLcs++;
            }
            else
            {
                nSpsLcs++;
            }
        }
        if ((m_prioToSps && nSpsLcs > 0) || (!m_prioToSps && nDynLcs == 0 && nSpsLcs > 0))
        {
            dynamicGrant = false;
        }
    }
    else
    {
        dynamicGrant = lcgMap.begin()->second->IsLcDynamic(lcIdsbyPrio.rbegin()->second.front());
    }
    if (dynamicGrant)
    {
        allocationInfo.m_isDynamic = true;
        NS_LOG_DEBUG("Selected scheduling type: dynamic grant / per-PDU ");
    }
    else
    {
        allocationInfo.m_isDynamic = false;
        NS_LOG_DEBUG("Selected scheduling type: SPS");
    }

    allocationInfo.m_harqEnabled =
        lcgMap.begin()->second->IsHarqEnabled(lcIdsbyPrio.rbegin()->second.front());

    // Remove all LCs that don't have the selected scheduling type
    uint16_t nLcs = 0;
    uint16_t nRemainingLcs = 0;
    uint8_t lcIdOfRef = 0;
    for (auto itlcIdsbyPrio = lcIdsbyPrio.rbegin(); itlcIdsbyPrio != lcIdsbyPrio.rend();
         ++itlcIdsbyPrio)
    {
        uint8_t lowestLcId = std::numeric_limits<uint8_t>::max();
        for (auto itLcs = itlcIdsbyPrio->second.begin(); itLcs != itlcIdsbyPrio->second.end();)
        {
            nLcs++;
            if (lcgMap.begin()->second->IsLcDynamic(*itLcs) != dynamicGrant)
            {
                itLcs = itlcIdsbyPrio->second.erase(itLcs);
            }
            else
            {
                if (*itLcs < lowestLcId)
                {
                    lowestLcId = *itLcs;
                }
                ++itLcs;
                nRemainingLcs++;
            }
        }
        if (itlcIdsbyPrio->second.size() == 0)
        {
            itlcIdsbyPrio = std::reverse_iterator(lcIdsbyPrio.erase(--itlcIdsbyPrio.base()));
        }

        if (lowestLcId != std::numeric_limits<uint8_t>::max() && lcIdOfRef == 0)
        {
            lcIdOfRef = lowestLcId;
        }
    }
    if (!dynamicGrant)
    {
        for (auto itlcIdsbyPrio = lcIdsbyPrio.begin(); itlcIdsbyPrio != lcIdsbyPrio.end();
             ++itlcIdsbyPrio)
        {
            for (auto itLcs = itlcIdsbyPrio->second.begin(); itLcs != itlcIdsbyPrio->second.end();)
            {
                if (lcgMap.begin()->second->GetLcRri(*itLcs) !=
                    lcgMap.begin()->second->GetLcRri(lcIdOfRef))
                {
                    itLcs = itlcIdsbyPrio->second.erase(itLcs);
                    nRemainingLcs--;
                }
                else
                {
                    ++itLcs;
                }
            }
            if (itlcIdsbyPrio->second.size() == 0)
            {
                itlcIdsbyPrio = lcIdsbyPrio.erase(itlcIdsbyPrio);
            }
        }

        allocationInfo.m_rri = lcgMap.begin()->second->GetLcRri(lcIdOfRef);
        m_reselCounter = GetRandomReselectionCounter(allocationInfo.m_rri);
        m_cResel = m_reselCounter * 10;
        NS_LOG_DEBUG("SPS Reselection counters: m_reselCounter " << +m_reselCounter << " m_cResel "
                                                                 << m_cResel);
    }
    allocationInfo.m_priority = lcgMap.begin()->second->GetLcPriority(lcIdOfRef);
    allocationInfo.m_castType = lcgMap.begin()->second->GetLcCastType(lcIdOfRef);
    NS_LOG_DEBUG("Number of LCs to attempt allocation for the selected destination: "
                 << nRemainingLcs << "/" << nLcs << ". LcId of reference " << +lcIdOfRef);

    // 2. Allocation of sidelink resources（核心修改区域）
    NS_LOG_DEBUG("Getting resources");
    std::map<uint8_t, std::vector<uint8_t>> selectedLcs = lcIdsbyPrio;
    std::queue<std::vector<uint8_t>> allocQueue;
    uint32_t bufferSize = 0;
    uint32_t nLcsInQueue = 0;
    uint32_t candResoTbSize = 0;
    uint8_t dstMcs = itDstInfo->second->GetDstMcs();
    uint16_t symbolsPerSlot = 9;
    uint16_t subChannelSize = GetMac()->GetNrSlSubChSize();
    auto rItSelectedLcs = selectedLcs.rbegin();

    CarlaTxCommand manualCmd;
    bool hasManualCmd = false;
    std::map<uint32_t, std::queue<CarlaTxCommand>>::iterator dstCmdIt;
    {
        std::lock_guard<std::mutex> lock(m_cmdMutex);
        // 按选中的目标ID查询命令map, 获取对应队列
        dstCmdIt = m_carlaTxCommandsByDst.find(dstIdSelected);
        if (dstCmdIt != m_carlaTxCommandsByDst.end() && !dstCmdIt->second.empty())
        {
            manualCmd = dstCmdIt->second.front(); // 获取队列头命令
            uint32_t srcL2Id = GetMac()->GetSrcL2Id();
            if (manualCmd.srcL2Id == srcL2Id)
            {
                // std::cout << "[DEBUG] Manual scheduling command matched: srcL2Id=" << manualCmd.srcL2Id 
                //             << ", dstL2Id=" << manualCmd.dstL2Id << ", current maxDataSize=" << manualCmd.maxDataSize << std::endl;
                NS_LOG_DEBUG("Manual scheduling command matched: srcL2Id=" << manualCmd.srcL2Id 
                                << ", dstL2Id=" << manualCmd.dstL2Id 
                                << ", current maxDataSize=" << manualCmd.maxDataSize);
                hasManualCmd = true;
            }
        }
    }
    while (selectedLcs.size() > 0)
    {
        allocQueue.emplace(rItSelectedLcs->second);
        uint32_t currBufferSize = 0;
        for (auto& itLc : rItSelectedLcs->second)
        {
            currBufferSize = currBufferSize + lcgMap.begin()->second->GetTotalSizeOfLC(itLc);
            // std::cout << "currBufferSize: " << currBufferSize << std::endl;
            NS_LOG_DEBUG("currBufferSize: " << currBufferSize);
        }
        nLcsInQueue = nLcsInQueue + rItSelectedLcs->second.size();
        bufferSize = bufferSize + currBufferSize;

        // Calculate number of needed subchannels
        uint16_t lSubch = 0;
        uint32_t tbSize = 0;
        uint32_t totalSubCh = GetTotalSubCh();
        // std::cout << "totalSubCh: " << totalSubCh << std::endl;
        if(hasManualCmd) {
            lSubch = manualCmd.slSubchannelSize;
            tbSize = CalculateTbSize(GetAmc(), dstMcs, symbolsPerSlot, lSubch, subChannelSize);
            // std::cout << "hasManualCmd. lSubch: " << lSubch << ", tbSize: " << tbSize << ", GetTotalSubCh: " << totalSubCh << std::endl;
            NS_LOG_DEBUG("hasManualCmd. lSubch: " << lSubch << ", tbSize: " << tbSize << ", GetTotalSubCh: " << totalSubCh);
        } else {
            do {
                lSubch++;
                tbSize = CalculateTbSize(GetAmc(), dstMcs, symbolsPerSlot, lSubch, subChannelSize);
            } while (tbSize < bufferSize + 5 && lSubch < totalSubCh);
            // std::cout << "lSubch: " << lSubch << ", tbSize: " << tbSize << ", GetTotalSubCh: " << totalSubCh << std::endl;
            NS_LOG_DEBUG("lSubch: " << lSubch << ", tbSize: " << tbSize << ", GetTotalSubCh: " << totalSubCh);
        }

        NS_LOG_DEBUG("Trying " << nLcsInQueue << " LCs with total buffer size of " << bufferSize
                               << " bytes in " << lSubch << " subchannels for a TB size of "
                               << tbSize << " bytes");

        NrSlUeMac::NrSlTransmissionParams params{lcgMap.begin()->second->GetLcPriority(lcIdOfRef),
                                                 lcgMap.begin()->second->GetLcPdb(lcIdOfRef),
                                                 lSubch,
                                                 lcgMap.begin()->second->GetLcRri(lcIdOfRef),
                                                 m_cResel};
        std::list<SlResourceInfo> filteredReso = FilterTxOpportunities(sfn,
            GetMac()->GetCandidateResources(sfn, params),
            lcgMap.begin()->second->GetLcRri(lcIdOfRef),
            m_cResel);
        // std::cout << "filteredReso size: " << filteredReso.size() << std::endl;
        if (filteredReso.empty())
        {
            NS_LOG_DEBUG("Resources not found");
            break;
        }

        if (hasManualCmd)
        {
            NS_LOG_DEBUG("Manual scheduling triggered: srcL2Id=" << manualCmd.srcL2Id 
                                                            << ", dstL2Id=" << manualCmd.dstL2Id
                                                            << ", subchannel[" << manualCmd.slSubchannelStart 
                                                            << "-" << manualCmd.slSubchannelStart + manualCmd.slSubchannelSize - 1 << "]");
            if (manualCmd.slSubchannelStart + manualCmd.slSubchannelSize > totalSubCh)
            {
                std::cerr << "[WARN] Manual subchannel out of range! Total subchannels: " << totalSubCh 
                          << ", requested: " << manualCmd.slSubchannelStart + manualCmd.slSubchannelSize << std::endl;
                break; // 子信道越界, 放弃本次调度
            }
            SlResourceInfo manualRes = *filteredReso.begin(); // 复制第一个资源作为模板
            filteredReso.clear(); // 清空原有资源列表
            manualRes.slSubchannelStart = manualCmd.slSubchannelStart;
            manualRes.slSubchannelLength = manualCmd.slSubchannelSize;
            filteredReso.push_back(manualRes); // 添加手动资源

            NS_LOG_DEBUG("Manual resource configured: slSubchannelStart=" << +manualRes.slSubchannelStart
                                                                    << ", slSubchannelLength=" << +manualRes.slSubchannelLength
                                                                    << ", PSSCH symbols=" << manualRes.slPsschSymLength);
        }

        NS_LOG_DEBUG("Resources found");
        candResoTbSize = tbSize;
        candResources = filteredReso;
        rItSelectedLcs = std::reverse_iterator(selectedLcs.erase(--rItSelectedLcs.base()));
    }
    if (candResources.size() == 0)
    {
        NS_LOG_DEBUG("Unable to find resources");
        return 0;
    }
    allocationInfo.m_tbSize = candResoTbSize;
    NS_LOG_DEBUG("Destination L2 ID " << dstIdSelected << " got " << candResources.size()
                                      << " resources (of TB size " << candResoTbSize << ")"
                                      << " available to allocate " << nLcsInQueue
                                      << " LCs with total buffer size of " << bufferSize
                                      << " bytes");

    // 2.2 Allocate the resources to logical channels
    uint32_t allocatedSize = 0;
    while (allocQueue.size() > 0)
    {
        uint32_t minBufferSize = std::numeric_limits<uint32_t>::max();
        uint32_t toServeBufferSize = 0;
        for (auto itLc : allocQueue.front())
        {
            if (lcgMap.begin()->second->GetTotalSizeOfLC(itLc) < minBufferSize)
            {
                minBufferSize = lcgMap.begin()->second->GetTotalSizeOfLC(itLc);
            }
        }
        toServeBufferSize = minBufferSize;
        if (allocQueue.front().size() * toServeBufferSize >
            candResoTbSize - allocatedSize - 5)
        {
            toServeBufferSize =
                std::floor((candResoTbSize - allocatedSize - 5) / allocQueue.front().size());
        }
        if (toServeBufferSize > 0)
        {
            for (auto itLc : allocQueue.front())
            {
                SlRlcPduInfo slRlcPduInfo(itLc, toServeBufferSize);
                allocationInfo.m_allocatedRlcPdus.push_back(slRlcPduInfo);
                NS_LOG_INFO("LC ID " << +itLc << " Dst L2ID " << dstIdSelected << " allocated "
                                     << toServeBufferSize << " bytes");
                allocatedSize = allocatedSize + toServeBufferSize;
            }
        }
        else
        {
            break;
        }

        allocQueue.pop();
    }

    // std::cout << "[DEBUG] Total allocated size: " << allocatedSize << " bytes" << std::endl;
    NS_LOG_DEBUG("Total allocated size: " << allocatedSize << " bytes");
    // 更新手动命令的剩余数据大小
    if (hasManualCmd){
        std::lock_guard<std::mutex> lock(m_cmdMutex);
        CarlaTxCommand& cmdToUpdate = dstCmdIt->second.front();
        // std::cout << "[DEBUG] Updated maxDataSize for cmd: (" << cmdToUpdate.maxDataSize << " --> "
        //             << (cmdToUpdate.maxDataSize - int(allocatedSize)) << ") bytes" << std::endl;
        NS_LOG_DEBUG("Updated maxDataSize for cmd: (" << cmdToUpdate.maxDataSize << " --> "
                    << (cmdToUpdate.maxDataSize - int(allocatedSize)) << ") bytes");
        cmdToUpdate.maxDataSize -= int(allocatedSize);
        // 小于 0 则 pop 队列头命令
        if (cmdToUpdate.maxDataSize <= 0) {
            // std::cout << "[DEBUG] maxDataSize <= 0, pop command: srcL2Id=" << cmdToUpdate.srcL2Id<< ", dstL2Id=" << cmdToUpdate.dstL2Id << std::endl;
            NS_LOG_DEBUG("maxDataSize <= 0, pop command: srcL2Id=" << cmdToUpdate.srcL2Id<< ", dstL2Id=" << cmdToUpdate.dstL2Id);
            dstCmdIt->second.pop();
        }
    }
    return dstIdSelected;
}

}