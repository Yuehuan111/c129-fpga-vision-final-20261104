# Code 页面文件说明

本仓库从最终交付 ZIP 展开全部 547 个文件，仅去掉最外层包装目录。543 个文件按原始字节保存，4 个历史日志改用 gzip 无损压缩；没有修改工程源码、算法、参数、PDS 工程、上位机或位流。

工程入口：`C129_compiled_final_20261031/`。完整原 ZIP 保留在 [Release 附件](https://github.com/Yuehuan111/c129-fpga-vision-final-20261104/releases/tag/C129-final-20261104)。

## 大日志

以下日志原始大小超过 100 MiB，Code 中在原路径末尾增加 `.gz`。正常启动上位机不需要预先解压这些历史日志。需要复核历史采样时，可使用支持 gzip 的解压软件解压，恢复为原 `.jsonl`。

| Code 文件 | 原大小（字节） |
| --- | ---: |
| `C129_compiled_final_20261031/host/logs/20261103_190416_715543.jsonl.gz` | 130183922 |
| `C129_compiled_final_20261031/host/logs/20261103_195816_696104.jsonl.gz` | 150666769 |
| `C129_compiled_final_20261031/host/logs/20261103_202506_978390.jsonl.gz` | 118364007 |
| `C129_compiled_final_20261031/host/logs/20261103_203518_521020.jsonl.gz` | 135101126 |

## 校验

`code_import_manifest.json` 逐项记录原 ZIP 路径、Code 路径、原始/存储大小、SHA256 和包装方式。对原样文件直接核对 SHA256；对 `.gz` 文件解压后核对 `original_sha256`。所有 gzip 文件已回读解压并核验原始 SHA256 一致。

`package_manifest.json` 仍保留原 ZIP 的完整条目清单；`SHA256SUMS.txt` 对应原 ZIP。原包和本仓库展开版本属于同一份最终交付内容。
