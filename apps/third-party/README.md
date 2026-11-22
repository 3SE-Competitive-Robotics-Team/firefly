# 第三方项目

这里用 Git submodule 固定 Firefly 使用的第三方项目版本。目前的飞控固件来自 [3SE 的 Betaflight fork](betaflight)。

我们只为实现板上功能修改原厂已有文件，例如接入 BMI270 或处理 Firefly 串口协议。构建、测试、CI 和工具链安装沿用原厂入口。Firefly 需要自己的流程时，在 monorepo 中新增独立入口，调用原厂命令。

## 编译 Matrix BMI270 固件

Matrix 固件统一用 Apple `container` 编译。Betaflight 的 `make` 命令在 `linux/amd64` 容器内执行，使用 Linux 版 Arm GNU Toolchain 13.3.Rel1。在 Firefly 仓库根目录运行下列命令。

```sh
git submodule update --init apps/third-party/betaflight
recipe=apps/third-party/betaflight-build
container build --platform linux/amd64 -t firefly-betaflight -f "$recipe/Containerfile" "$recipe"
container run --rm --platform linux/amd64 -u "$(id -u):$(id -g)" -v "$PWD:/src" firefly-betaflight
```

镜像建一次即可。后续编译只需执行最后一行。首次编译会把工具链下载到子模块的 `tools/`。刷写、交付和 SHA-256 校验均使用 `apps/third-party/betaflight/obj/container/` 下的 HEX。

## 验证串口协议

在 macOS 上用系统 Clang 运行 Firefly 串口协议单元测试。

```sh
make -C apps/third-party/betaflight/src/test -f ../../../betaflight-build/tests.mk \
    CC=/usr/bin/clang CXX=/usr/bin/clang++ test
```

日常检查可直接看 [pilot 和 RootSLAM 的台架日志](../../docs/development.md#验证-matrix-imu)。要单独检查固件，在 Orange Pi 上停止 pilot，将已上锁的 Matrix 接到 USB Host。从板上的源码目录运行以下脚本，或将脚本复制到板上并使用对应路径。设备名以实际枚举结果为准。

```sh
ffctl process stop pilot
python3 apps/third-party/betaflight-build/read_imu.py /dev/ttyACM0
```

脚本每 250 ms 发送心跳，每秒显示一个样本。按 Ctrl-C 退出，等一秒多，再用 `ffctl process start pilot` 恢复采集。
