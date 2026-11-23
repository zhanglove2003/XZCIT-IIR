//! 构建脚本：aarch64 musl 交叉编译的链接器指定（cwd 无关）。
//!
//! 问题：rustc 对 aarch64-unknown-linux-musl 的默认链接器解析落到宿主
//! `cc`（gcc），AArch64 errata 选项 `--fix-cortex-a53-843419` 传给 x86_64
//! 宿主 GNU ld 被拒（unrecognized option）。
//!
//! 为什么不用 .cargo/config.toml：cargo 的配置发现基于**当前工作目录**
//! 的祖先链（cargo 1.100 仍如此，`--manifest-path` 不参与该搜索），调用方
//! cwd 不在本项目内时 config 不会被读取。
//!
//! 为什么不能用 build script 直接指定链接器：cargo 1.100 不存在
//! `cargo::rustc-linker` 指令（新式报 Unknown key，旧式 `cargo:rustc-linker`
//! 被当作未知元数据静默丢弃——两者均实测过）。build script 能做的只有
//! 向链接器命令行追加参数（cargo:rustc-link-arg），而最终链接命令的
//! 头部永远是目标默认链接器 `cc`。
//!
//! 因此用 gcc/clang 通用的 `-B<dir>/` 前缀机制：driver 搜索子程序
//! （ld/as/...）时**先查 -B 前缀**，命中 `<dir>/ld` 即用它作链接器。
//! build.rs 在 OUT_DIR 里生成这个 `ld` 垫片（exec 到 rustc sysroot 内
//! 捆绑的 rust-lld，`-flavor gnu` 使其按 GNU ld 语义解析参数）：
//! - lld 完整支持 --fix-cortex-a53-843419；
//! - musl target 的 std 自带 self-contained musl libc（crt*.o/libc.a 在
//!   rustlib/aarch64-unknown-linux-musl/lib/self-contained），链接无需任何
//!   外部交叉工具链；
//! - musl 目标默认 crt-static → 全静态产物，满足部署铁律。
//! 垫片随 crate 走（OUT_DIR 由 cargo 提供、路径每次构建现算），与 cwd
//! 和 .cargo 配置发现完全解耦。
//!
//! 条件限定：只对 aarch64 + linux + musl 目标生效；host 构建及
//! --fake 本机调试不受影响。sysroot 内找不到 rust-lld 时**不注入任何
//! 参数**（构建退回默认 cc 行为，与改动前一致）并告警，绝不硬失败。

use std::fs;
use std::os::unix::fs::PermissionsExt;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let is_aarch64_musl = std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("aarch64")
        && std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux")
        && std::env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("musl");
    if !is_aarch64_musl {
        return;
    }

    // 解析 rustc sysroot（RUSTC 由 cargo 注入 build script，跟随实际编译器）
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".into());
    let sysroot = match Command::new(&rustc).args(["--print", "sysroot"]).output() {
        Ok(out) if out.status.success() => String::from_utf8_lossy(&out.stdout).trim().to_string(),
        _ => {
            eprintln!("web_next build.rs: rustc --print sysroot failed; default linker unchanged");
            return;
        }
    };
    let host = std::env::var("HOST").unwrap_or_default(); // cargo 注入宿主三元组
    let lld = PathBuf::from(&sysroot).join(format!("lib/rustlib/{host}/bin/rust-lld"));
    if !lld.is_file() {
        eprintln!(
            "web_next build.rs: bundled rust-lld not found at {}; default linker unchanged",
            lld.display()
        );
        return;
    }

    // 在 OUT_DIR 生成 ld 垫片：<prefix>ld 是 gcc/clang -B 前缀搜索的名字。
    // 实测 gcc 13.3 的 collect2 会在参数里带宿主原生 emulation
    // `-m elf_x86_64`（位于我们注入参数之后），lld 按 last-wins 取 emulation，
    // 导致 --fix-cortex-a53-843419 被判「only supported on AArch64」。
    // 全部输入对象均为 aarch64（musl 交叉产物），故丢弃一切 `-m <值>` 对，
    // 末尾强制 `-m aarch64linux`；gcc 其余传参（-plugin 等）lld 按 GNU 兼容
    // 语义接受/忽略。
    let shim_dir =
        PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR set by cargo")).join("lld_shim");
    fs::create_dir_all(&shim_dir).expect("create lld shim dir");
    let shim = shim_dir.join("ld");
    let script = "#!/bin/bash\n\
        args=()\n\
        while [ $# -gt 0 ]; do\n\
        \x20 case \"$1\" in\n\
        \x20   -m) if [ $# -ge 2 ]; then shift 2; else shift; fi ;;\n\
        \x20   *) args+=(\"$1\"); shift ;;\n\
        \x20 esac\n\
        done\n\
        exec \""
        .to_string()
        + &lld.display().to_string()
        + "\" -flavor gnu -m aarch64linux \"${args[@]}\"\n";
    fs::write(&shim, script).expect("write lld shim");
    let mut perms = fs::metadata(&shim).expect("shim metadata").permissions();
    perms.set_mode(0o755);
    fs::set_permissions(&shim, perms).expect("chmod shim");

    // -B 是前缀语义：必须以 / 结尾，driver 拼接 {prefix}ld 后命中垫片
    let mut prefix = shim_dir.display().to_string();
    if !prefix.ends_with('/') {
        prefix.push('/');
    }
    println!("cargo:rustc-link-arg=-B{prefix}");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=RUSTC");
}
