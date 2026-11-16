# 当前标定与成功电脑参数快照

当前入口为[latest.json](latest.json)，固定指向[2026-11-16成功演示电脑导出](2026-11-16-success-demo/README.md)。旧的11月15日归档保留为历史记录。

**拉取代码不会导入本机标定或经验参数。** 程序优先读取`~/.evoarm/rsi/calibrations/bench-layout-v2/active-session.json`选中的revision；单球与挑战策略分别按同一标定指纹加载。另一个用户、另一份活动指针或不同策略，即使在相同main代码下也会得到不同参数。

本快照仅适用于用户确认的同一套机械臂、相机、底座与桌面布局，位置保持不变。设备位置改变时使用新的五点标定，不能把此快照当作新布局的测量。

## 直接传给同事的ZIP

[下载独立迁移包](transfers/evoarm-success-demo-transfer-20261116.zip)（约556KB），[SHA256校验值](transfers/evoarm-success-demo-transfer-20261116.zip.sha256)。它包含本次导出的原始数据、清单和可独立运行的备份导入脚本。把ZIP交给同事即可，不必另外手工寻找本机文件，不安装或升级Python环境。

解压进入`evoarm-success-demo-transfer`目录，按包内README操作：先`python3 tools/install_layout.py compare`；结束任务并停止网页服务后运行`python3 tools/install_layout.py install`；用原方式重新启动网页，再运行`python3 tools/install_layout.py compare --controller http://127.0.0.1:8765`。

没有下载入口时，可在已有项目中只fetch保存此包的提交，再用`git show <提交号>:calibrations/bench-layout-v2/transfers/evoarm-success-demo-transfer-20261116.zip > ~/Downloads/evoarm-success-demo-transfer-20261116.zip`导出；这不切换或修改当前代码分支。配置包只迁移6个在用数据文件，程序代码继续使用main。

维护者可用`python -m evoarm_rsi.layout_snapshot package --output /tmp/evoarm-success-demo-transfer-20261116.zip`重新生成独立包。更新导入脚本或固定快照后应同步更新ZIP与SHA256，测试会检查两者一致。

## 同事电脑复现

在更新后的main项目目录、原来的普通用户Python环境下执行；仅相机进程继续沿用原权限方式。以下工具不连接机械臂、不切换扭矩、不调用视觉模型。

先只读检查仓库数据及本机差异：

```sh
python -m evoarm_rsi.layout_snapshot verify
python -m evoarm_rsi.layout_snapshot compare
```

`compare`的`calibration_matches`表示当前活动文件与成功电脑相同；`local_policies`分别显示两个模式的版本和参数是否一致；`matches_snapshot`还要求安装文件的原始SHA256全部一致。发现不同返回退出码1，但不修改文件。不存在的文件也会明确报告。

需要恢复成功电脑基线时，先结束任务并停止网页服务，避免并发采点、学习写入和旧的待执行计划。无需发送复位命令；相机服务和模型配置可沿用。然后显式导入：

```sh
python -m evoarm_rsi.layout_snapshot install
```

导入只复制6个数据文件：活动指针、session.json、参考图、两种模式的策略、参考SDK内参。已有不同文件先原样备份到`~/.evoarm/rsi/layout_backups/<时间-标识>/`；先写数据、最后切活动指针，写入失败会回滚。原视频、历史成功执行记录、token、环境配置和串口设置均不安装。文件已一致时不写入、不新建备份。

如果导入报`experiments`目录Permission denied，先用`ls -ld "$HOME/.evoarm/rsi/experiments"`检查。旧相机服务在sudo下可能把顶层目录建为root所有；仅确认是这种情况时，执行`sudo chown "$(id -u):$(id -g)" "$HOME/.evoarm/rsi/experiments"`及`chmod u+rwx "$HOME/.evoarm/rsi/experiments"`，然后用普通用户重新导入。不要sudo整个导入程序或递归改整个用户目录。新导入工具会在任何配置写入前检查所有待写目录，相机服务也会把自己创建的`rsi/`和`experiments/`目录交还给实际用户。

使用原环境重新启动网页服务后，核对真正运行的服务：

```sh
python -m evoarm_rsi.layout_snapshot compare --controller http://127.0.0.1:8765
```

除本机文件外，这条命令只GET现有网页的标定状态和readiness；readiness可能读取串口反馈，不发送运动命令。核对`runtime.session_path_matches_local=true`、`runtime.fingerprint_matches_snapshot=true`、`matches_snapshot=true`，再由用户开始测试。报告给出服务启动时加载的`code_revision`与实际项目目录，不能只看终端的git分支。

`install`输出中的`source_runtime`是成功电脑的来源记录，并非接收电脑正在运行的进程。安装后文件SHA256一致，只能证明文件复制完成；需重启网页后运行带`--controller`的比较，才能核对服务实际加载的路径。ZIP不部署网页代码；若没有挑战升级页面，检查实际服务的项目目录/提交，更新并重启main网页服务，`/api/challenge/state`应返回`challenge_api_version=2`。

同一内容可以有不同提交号：成功电脑`4aad187`与原main`8d15d54`代码树相同。本次数据更新后的main还会包含新归档与此工具，因此提交号再次变化是正常的。

## 后续学习

成功电脑的当前单球策略为v64，挑战策略为v13；并非旧归档的v17。导入后继续学习会修改本机策略版本或参数，再次与固定归档比较时可能不相同，这不是自动回退或标定丢失。工具不会在启动或读取状态时自动覆盖这些新经验。

CLI可用`--home`对指定用户目录做离线检查或安装，用`--index`验证指定的已固定清单；默认均为当前用户与本仓库latest.json。修改其他电脑前先查看差异、保留备份，运行服务与数据目录必须属于同一用户。
