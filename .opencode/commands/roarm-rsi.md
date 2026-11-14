---
description: 进入机械臂 RSI Agent；无参数只读检查，go 启动抓放与失败回位
agent: roarm-rsi
model: bottlerocket/grok-4.7-build
subtask: false
---

使用 roarm-rsi 主Agent及其现有工具协议处理：$ARGUMENTS

先执行 python -m evoarm_rsi.agent_tools readiness，再读取 context，核对本次连接、控制服务目录和版本。不要使用旧会话的连接推断。armed=false不是阻止启动的理由；run会在完整检查后申请临时授权，网页没有额外解锁按钮。

参数为空或只要求状态时，仅做只读检查并说明结果，不开始抓放、复位、连接或切换扭矩。go或开始就是用户对默认任务的执行请求：抓取当前黄球、投放漏斗、失败后官方回位并调整，最多3轮、成功即停；继续沿用此前已经请求的任务范围。依次完成视觉子Agent复核、执行、结果分析、单项参数试验与记录。不得因armed=false或没有额外网页口令拒绝go，也不得只回复“已开始”而不调用工具。
