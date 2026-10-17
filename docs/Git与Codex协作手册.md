# Git 与 Codex 协作手册

本文供收到 `yyb-stm32-template.bundle` 的开发者使用。Bundle 包含仓库的提交历史；克隆后可在自己的电脑继续修改、提交和创建分支。Codex 可协助检查代码与文档，但硬件接线、供电和实机运动仍需操作者核实。

## 一 取得工程

将 Bundle 放到准备存放工程的目录，在 PowerShell 中执行：

```powershell
git clone .\yyb-stm32-template.bundle yyb-stm32-template
Set-Location .\yyb-stm32-template
git status
git log -3 --oneline
```

应看到工作区干净以及已有提交历史。请用克隆出的目录工作，不要在 Bundle 文件中直接修改。首次提交前，在本机设置真实的 Git 姓名和邮箱：

```powershell
git config user.name "你的姓名"
git config user.email "你的邮箱"
```

## 二 让 Codex 熟悉工程

在 Codex 中打开克隆出的 `yyb-stm32-template` 目录，先向它发送：

> 请先阅读 README.md、docs/移植检查表.md、引脚配置汇总.md、docs/转接板使用手册.md 和 USER/app_config.h。当前机构 PA3 控制升降、PA2 控制夹爪；源码中的云台和物块平台是旧名称。先给出将要修改的文件、依据和验证方法，再进行我提出的具体改动。不要把其他工程的引脚配置套入这个工程。

随后按一次一个任务提出需求，例如“只修改 USART1 的接收解析，并检查原有发送格式”“先检查 PA3 升降方向，再调整脉宽范围”。涉及机械运动时，应向 Codex提供实际舵机型号、供电、方向和机构限位；不要只凭源码中的角度换算放大行程。

## 三 构建与验证

Keil µVision 5 打开 `USER/Template.uvprojx`，目标为 STM32F407ZG。默认 `USER/app_config.h` 只开启 `APP_SAFE_IDLE=1`，上电保持空闲。启用某个测试入口时，先把空闲模式设为 `0`，再只打开一个应用模式。执行 Rebuild 后记录 `Error(s)` 和 `Warning(s)` 数量；软件构建通过仍需在对应板卡上实测。

可让 Codex 协助检查改动：

> 请对比本次 Git 改动，检查模式宏是否互斥、Keil 工程是否包含新增源文件、PA2/PA3 机械用途与文档是否一致，并列出尚未实测的部分。不要把语法检查说成整板验证。

## 四 保存并交流改动

每完成一个能独立验证的改动，执行：

```powershell
git status
git diff --check
git add -A
git commit -m "说明本次改动"
```

发回给原开发者时，可在仓库目录创建包含全部分支和提交的新 Bundle：

```powershell
git bundle create ..\yyb-stm32-template-updated.bundle --all
git bundle verify ..\yyb-stm32-template-updated.bundle
```

对方可克隆这个更新版 Bundle。若需要长期双向协作，可以在核实第三方代码许可和发布范围后配置双方可访问的 Git 远程仓库，再用 `git push` / `git pull` 同步；Bundle 本身不会自动同步。

## 五 文件边界

- 固件入口与模式：`USER/main.c`、`USER/app_config.h`。
- 引脚和移植说明：`引脚配置汇总.md`、`docs/移植检查表.md`。
- 上位机通信：`SYSTEM/Serial.c` 的 USART1，PA9 TX、PA10 RX，9600 8N1。接收使用中断，实际数据包格式要对照源码。
- 当前装配与供电：`docs/转接板使用手册.md`。连接器针序需查实物或原理图，不可从 MCU 引脚号推断。
- `OBJ/` 和 Keil 用户配置由 `.gitignore` 排除，不要提交编译产物。
