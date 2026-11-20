# 用 CMake 打开 C50C 固件工程

这个目录可以直接作为 CMake 工程打开。默认配置对应现有 Makefile 的四轮 ROS 构建配置。
原有 Makefile 和 Keil 工程仍可使用。CMake 只编译固件，不会连接板子或自动烧录。

## 准备

安装 CMake 3.21 或更新版本，以及 Arm GNU Toolchain。确认以下命令可以运行：

```sh
cmake --version
arm-none-eabi-gcc --version
```

编译器、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size` 需要在 `PATH` 中。
如果使用其他 Arm GCC 工具链配置，可不使用本工程预设，并在配置 CMake 时指定自己的
`CMAKE_TOOLCHAIN_FILE`。

## VS Code

1. 安装 CMake Tools 和 C/C++ 扩展。
2. 用“打开文件夹”打开本目录。
3. 选择 `stm32-arm` 配置，执行 `CMake: Configure`，再执行 `CMake: Build`。

根目录的 `CMakeLists.txt` 默认加载本工程的 Arm GCC 工具链文件，并生成
`compile_commands.json`，供编辑器识别宏、头文件和源码。

## CLion

1. 用“Open”打开本目录，选择作为项目打开。
2. 等待 CMake 加载。找不到交叉编译器时，请把 Arm GNU Toolchain 的 `bin` 目录加入
   CLion 所用环境的 `PATH`，然后重新加载 CMake。
3. 选择 `Diff_Car` 目标并构建。

## 命令行构建

```sh
cmake --preset stm32-arm
cmake --build --preset stm32-arm -j
```

成功后，构建目录包含 `Diff_Car.elf`、`Diff_Car.hex`、`Diff_Car.bin` 和
`Diff_Car.map`。这些文件只是构建结果；不能据此认定它们与板上固件逐字节一致。

## 中文注释显示

源码包含 UTF-8 和 GBK/GB18030 文件，不要统一转换并保存整个工程。

- VS Code：对乱码文件使用状态栏编码菜单，选择“通过编码重新打开”，再选择 GBK 或 GB18030。
- CLion：对乱码文件使用 `File > File Properties > File Encoding`，选择 GBK 或 GB18030 并重新加载；不要选择转换原文件。
- `SYSTEM/sys/sys.c` 同时包含两种编码，单一编码不能完整显示全部中文；请保留原文件字节。

此 CMake 配置不修改任何 STM32 控制逻辑，也不默认启用 `CHASSIS_PWM_TEST`。
