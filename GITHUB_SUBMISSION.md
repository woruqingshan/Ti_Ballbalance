# GitHub 提交说明

建议将本目录整体放到仓库：

`firmware/ti/BallBalanceControl_MSPM0_v0_1/`

## 应提交

- `keil/*.uvprojx`
- `keil/*.sct`
- `src/**`
- `generated/ti_msp_dl_config.c/.h`
- `config/*.syscfg`
- `tools/**`
- `host_tests/**`
- `docs/**`
- `.gitignore`
- `README.md`

## 不提交

Keil 的 `Objects/`、`Listings/`、AXF/HEX/MAP、个人调试配置、日志和 Python 缓存。SDK 本体不复制到仓库，项目通过固定路径引用：

`D:\TI\M0_SDK\mspm0_sdk_2_02_00_05`

## 新仓库

```powershell
cd D:\path\to\BallBalanceControl_MSPM0_v0_1
git init
git branch -M main
git add .
git status
git commit -m "feat(ti): add MSPM0G3507 ball balance control v0.1"
git remote add origin https://github.com/<owner>/<repo>.git
git push -u origin main
```

## 加入现有 VisionControlandWirelessComm

```powershell
cd D:\path\to\VisionControlandWirelessComm
New-Item -ItemType Directory -Force firmware\ti | Out-Null
Copy-Item -Recurse -Force D:\path\to\BallBalanceControl_MSPM0_v0_1 firmware\ti\
git add firmware/ti/BallBalanceControl_MSPM0_v0_1
git status
git commit -m "feat(ti): add visual ball-balance controller v0.1"
git push origin main
```

## 提交前检查

```powershell
git status --short
git check-ignore -v firmware\ti\BallBalanceControl_MSPM0_v0_1\keil\Objects\dummy.o
Get-ChildItem -Recurse firmware\ti\BallBalanceControl_MSPM0_v0_1 | Select-String "D:\\TI\\M0_SDK\\mspm0_sdk_2_02_00_05"
```

不要提交本地生成的 HEX 作为源代码基线。需要发布可烧录固件时，使用 GitHub Release 或单独的 `artifacts/` 发布流程。
