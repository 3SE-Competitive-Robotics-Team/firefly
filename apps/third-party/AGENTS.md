# 第三方项目协作约定

- 修改第三方已有文件时，将改动限于固件运行所需的实现。板级差异优先使用上游提供的配置扩展点。
- 保留原厂的 Makefile、工具链安装脚本、测试入口和 CI 配置。Firefly 专用的构建、测试、容器和 CI 流程以新文件放在 monorepo 中，调用上游公开入口。
- Matrix 飞控固件统一用 Apple `container` 的 `linux/amd64` 镜像编译。刷写、交付和校验均使用子模块 `obj/container/` 下的 HEX。构建命令见 [README](README.md#编译-matrix-bmi270-固件)。
- 更新 submodule 指针前，对照所基于的上游提交检查文件差异，确认每个改动过的原厂文件都服务于板上功能。
