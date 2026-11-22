# 第三方项目

这里用 Git submodule 固定 Firefly 使用的第三方项目版本。目前的飞控固件来自 [3SE 的 Betaflight fork](betaflight)。

我们只为实现板上功能修改原厂已有文件，例如接入 BMI270 或处理 Firefly 串口协议。构建、测试、CI 和工具链安装沿用原厂入口。Firefly 需要自己的流程时，在 monorepo 中新增独立入口，调用原厂命令。

## 编译 Matrix BMI270 固件

在 Firefly 仓库根目录运行。容器使用该版 Betaflight 原生支持的 Linux x86-64 工具链。

```sh
git submodule update --init apps/third-party/betaflight
recipe=apps/third-party/betaflight-build
container build --platform linux/amd64 -t firefly-betaflight -f "$recipe/Containerfile" "$recipe"
container run --rm --platform linux/amd64 -u "$(id -u):$(id -g)" -v "$PWD:/src" firefly-betaflight
```

镜像建一次即可。后续编译只需执行最后一行。首次编译会下载 Arm GNU Toolchain 13.3.Rel1，缓存在子模块的 `tools/`。HEX 输出到 `apps/third-party/betaflight/obj/container/`。

在 macOS 上用系统 Clang 运行 Firefly 串口协议单元测试。

```sh
make -C apps/third-party/betaflight/src/test -f ../../../betaflight-build/tests.mk \
    CC=/usr/bin/clang CXX=/usr/bin/clang++ test
```
