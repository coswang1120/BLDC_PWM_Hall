# 更新日誌

## [年-月-日]
### 修改
- 調整 VS Code Git Graph 設定
- 設定 git-graph.maxDepthOfRepoSearch 為 2，以確保能正確掃描子資料夾中的 Git 儲存庫
- 修改 launch.json 配置以支援 JTAG+SWD 除錯
  - 更改 interface 設定為 stlink.cfg
  - 新增 openOCDLaunchCommands 設定使用 SWD 模式
  - 保留 STM32F1 系列的目標配置
