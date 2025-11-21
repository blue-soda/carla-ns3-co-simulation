#include "nr-sl-ue-mac-scheduler-cluster.h"

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

NS_LOG_COMPONENT_DEFINE("NrSlUeMacSchedulerCluster");
NS_OBJECT_ENSURE_REGISTERED(NrSlUeMacSchedulerCluster);
// 注册调度器TypeID
TypeId
NrSlUeMacSchedulerCluster::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::NrSlUeMacSchedulerCluster")
        .SetParent<NrSlUeMacSchedulerFixedMcs>()
        .AddConstructor<NrSlUeMacSchedulerCluster>()
        .AddAttribute("NumChannels", "Number of subchannels (K)",
                      UintegerValue(10),
                      MakeUintegerAccessor(&NrSlUeMacSchedulerCluster::m_numChannels),
                      MakeUintegerChecker<uint8_t>(1, 20))
        .AddAttribute("SinrThreshold", "SINR threshold for conflict detection",
                      DoubleValue(6.0), // 6dB约对应MCS 10的解调需求
                      MakeDoubleAccessor(&NrSlUeMacSchedulerCluster::m_sinrThreshold),
                      MakeDoubleChecker<double>(0.0, 20.0))
        .AddAttribute("MaxTxPower", "Maximum transmission power (W)",
                      DoubleValue(0.1), // 0.1W=20dBm, 符合V2X功率标准
                      MakeDoubleAccessor(&NrSlUeMacSchedulerCluster::m_maxTxPower),
                      MakeDoubleChecker<double>(0.01, 1.0))
        .AddAttribute("NoisePower", "Noise power (W)",
                      DoubleValue(1e-17), // 简化噪声模型
                      MakeDoubleAccessor(&NrSlUeMacSchedulerCluster::m_noisePower),
                      MakeDoubleChecker<double>(1e-19, 1e-15));
    return tid;
}


// CARLA添加传输指令(线程安全)
void
NrSlUeMacSchedulerCluster::AddCarlaTxCommand(const CarlaTxCommand& cmd)
{
    NS_LOG_FUNCTION(this << cmd.srcL2Id << cmd.dstL2Id << cmd.subchannel_start << cmd.subchannel_num);
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_carlaTxCommands.push_back(cmd);
}

// 实现: 清空已完成的指令
void
NrSlUeMacSchedulerCluster::ClearCompletedCommands()
{
    NS_LOG_FUNCTION(this);
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_carlaTxCommands.clear();
}

// 重写调度触发函数: 优先执行CARLA指令
void
NrSlUeMacSchedulerCluster::DoSchedNrSlTriggerReq(const SfnSf& sfn)
{
    NS_LOG_FUNCTION(this << sfn);

    // 步骤1: 优先执行CARLA下发的传输指令(核心)
    ExecuteCarlaCommands(sfn);

    // // 步骤2:(可选)保留NS-3自主调度逻辑, 作为降级方案
    // // 若CARLA无指令, 执行NS3端分簇调度
    // if (m_carlaTxCommands.empty())
    // {
    //     NS_LOG_DEBUG("No commands from CARLA, run NS3 local scheduling");
    //     // 调用NS-3端分簇调度逻辑
    //     // 收集链路→冲突图→分配RB→功率控制→创建Grant
    // }

    // 步骤3: 父类原有逻辑: 发布Grant到MAC层
    CheckForGrantsToPublish(sfn);
}

// 核心: 执行CARLA指令, 创建NS-3的传输Grant
void
NrSlUeMacSchedulerCluster::ExecuteCarlaCommands(const SfnSf& sfn)
{
    NS_LOG_FUNCTION(this << sfn);
    std::lock_guard<std::mutex> lock(m_cmdMutex);

    if (m_carlaTxCommands.empty())
    {
        NS_LOG_DEBUG("No Carla Tx commands to execute");
        return;
    }

    // 遍历所有CARLA指令, 逐一创建传输Grant
    for (const auto& cmd : m_carlaTxCommands)
    {
        // 校验指令有效性(子信道不越界、数据大小>0)
        uint8_t totalSubCh = GetTotalSubCh(); // 从父类获取总子信道数
        if (cmd.subchannel_start + cmd.subchannel_num > totalSubCh || cmd.dataSize == 0)
        {
            NS_LOG_WARN("Invalid Carla command: src=" << cmd.srcL2Id 
                << ", dst=" << cmd.dstL2Id << ", subchannel_start=" << (uint32_t)cmd.subchannel_start
                << ", subchannel_num=" << (uint32_t)cmd.subchannel_num);
            continue;
        }

        // 1. 构建NS-3的资源块分配参数(严格遵循CARLA指令)
        SlGrantResource grantRes;
        grantRes.sfn = sfn;          // 当前时隙
        grantRes.slPsschSubChStart = cmd.subchannel_start; // CARLA指定的子信道起始索引
        grantRes.slPsschSubChLength = cmd.subchannel_num;     // CARLA指定的子信道数量
        grantRes.slPsschSymLength = 14;      // 每个时隙14个符号(NS-3默认)

        // 创建SlResourceInfo对象而不是SlGrantResource
        std::list<SlResourceInfo> slotAllocList;
        // 假设获取一些必要的参数（这里使用默认值或从其他地方获取）
        uint16_t numSlPscchRbs = 2; // 默认值，可能需要从配置中获取
        uint16_t slPscchSymStart = 0; // 默认值
        uint16_t slPscchSymLength = 2; // 默认值
        uint16_t slPsschSymStart = 3; // 默认值
        uint16_t slSubchannelSize = 10; // 默认值
        uint16_t slMaxNumPerReserve = 1; // 默认值
        uint8_t slPsfchPeriod = 1; // 默认值
        uint8_t slMinTimeGapPsfch = 1; // 默认值
        uint8_t slMinTimeGapProcessing = 1; // 默认值
        
        SlResourceInfo resourceInfo(numSlPscchRbs, slPscchSymStart, slPscchSymLength,
                                   slPsschSymStart, grantRes.slPsschSymLength,
                                   slSubchannelSize, slMaxNumPerReserve,
                                   slPsfchPeriod, slMinTimeGapPsfch,
                                   slMinTimeGapProcessing, sfn,
                                   cmd.subchannel_start, cmd.subchannel_num);
        slotAllocList.push_back(resourceInfo);

        // 2. 构建传输块信息(复用NS-3结构体)
        AllocationInfo allocInfo;
        allocInfo.m_priority = 10; // 优先级(CARLA可指定, 这里设为固定高优先级)
        allocInfo.m_isDynamic = true; // 动态调度(单次传输)
        allocInfo.m_harqEnabled = true; // 启用HARQ(可选, 提升可靠性)
        allocInfo.m_tbSize = cmd.dataSize; // CARLA指定的数据大小
        // 传输类型: 广播(dst=0)→ GroupCast, 单播→Unicast
        // 注意：这里使用Groupcast而不是GroupCast（小写的c）
        // 由于无法找到SidelinkInfo::CastType的确切定义，这里暂时注释掉
        // 在实际使用中，需要找到正确的枚举值或定义

        // 3. 确保目标L2 ID在NS-3的dstMap中(创建空的DstInfo, 仅用于链路管理)
        if (m_dstMap.find(cmd.dstL2Id) == m_dstMap.end())
        {
            CreateDstInfoForCarlaCmd(cmd.dstL2Id); // 辅助函数: 创建目标信息
        }

        // 4. 调用父类方法创建Grant, 触发数据传输
        // 由于修改了slotAllocList的类型，这里可以直接调用AttemptGrantAllocation
        // 但由于注释掉了allocInfo.m_castType的设置，可能需要进一步调整
        AttemptGrantAllocation(sfn, cmd.dstL2Id, slotAllocList, allocInfo);
        NS_LOG_DEBUG("Executed Carla command: src=" << cmd.srcL2Id 
            << ", dst=" << cmd.dstL2Id << ", Subchannel=[" << (uint32_t)cmd.subchannel_start 
            << "," << (uint32_t)(cmd.subchannel_start+cmd.subchannel_num-1) << "], dataSize=" << cmd.dataSize);
    }

    // 执行完成后清空指令(避免重复调度, 也可在CARLA端控制清空时机)
    m_carlaTxCommands.clear();
}

// 辅助函数: 为CARLA指令创建目标L2 ID的DstInfo(NS-3调度器需要)
void
NrSlUeMacSchedulerCluster::CreateDstInfoForCarlaCmd(uint32_t dstL2Id)
{
    NS_LOG_FUNCTION(this << dstL2Id);

    // 1. 创建目标信息结构体 (NrSlUeMacSchedulerDstInfo)
    auto dstInfo = std::make_shared<NrSlUeMacSchedulerDstInfo>(dstL2Id);

    // 2. 准备逻辑信道 (LC) 的配置参数
    NrSlUeCmacSapProvider::SidelinkLogicalChannelInfo lcParams;
    lcParams.lcId = 13;                     // 逻辑信道ID (Sidelink默认数据信道)
    lcParams.priority = 5;                  // 优先级
    lcParams.pqi = 9;                       // PC5 QoS Class Identifier (非GBR)
    lcParams.isGbr = false;                 // 非GBR承载
    lcParams.castType = SidelinkInfo::CastType::Unicast; // 单播类型
    // 其他参数如 mbr, gbr, harqEnabled, pdb, dynamic, rri 等可按需设置

    // 3. 创建逻辑信道 (LC) 对象
    // CreateLC 通常返回一个 std::shared_ptr<NrSlUeMacSchedulerLC>
    auto lc = CreateLC(lcParams);
    if (!lc)
    {
        NS_LOG_ERROR("CreateDstInfoForCarlaCmd: Failed to create Logical Channel (LC) for dstL2Id " << dstL2Id);
        return;
    }

    // 4. 创建并填充逻辑信道组 (LCG)
    uint8_t lcgId = 1; // 选择一个LCG ID，例如1
    NrSlLCGPtr lcg = std::make_unique<NrSlUeMacSchedulerLCG>(lcgId);

    // 使用 LCG 的 Insert 方法来添加 LC。
    // CreateLC 返回的是 shared_ptr，而 Insert 需要 unique_ptr。
    // 我们需要将 shared_ptr 转换为 unique_ptr。
    // 注意：只有当你确定没有其他 shared_ptr 指向这个 LC 对象时，才能安全地执行此操作。
    // 在这个上下文中，因为是我们刚刚创建的，所以是安全的。
    lcg->Insert(NrSlLCPtr(lc.release()));

    // 5. 将 LCG 插入到 DstInfo 中
    // NrSlUeMacSchedulerDstInfo::Insert 方法也需要一个右值引用。
    dstInfo->Insert(std::move(lcg));

    // 6. 将准备好的 DstInfo 加入到调度器的全局 dstMap 中
    m_dstMap[dstL2Id] = dstInfo;

    NS_LOG_DEBUG("CreateDstInfoForCarlaCmd: Successfully created DstInfo for L2 ID " << dstL2Id 
                 << " with LC ID " << static_cast<uint32_t>(lcParams.lcId)
                 << " in LCG ID " << static_cast<uint32_t>(lcgId));
}

}