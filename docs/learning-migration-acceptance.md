# Learning 改名与 Genesis 配置解读验收

记录日期：2026-10-14。范围：[系列迁移 Issue #1](https://github.com/huangkiki/sim-learning/issues/1)中的 Genesis 文档切片。中文正文见[配置解读](solver-reading-guide.md)，英文摘要位于同页，中英文 README 与课程导航均已连接。验证对象为本切片的工作树；本页不声明远端提交、PR 合并或运行验收。

## 交付范围与版本

- 现行标题、导航和系列链接统一为 Genesis Learning / Sim Learning；历史 Issue URL 更新到新仓库路径，Issue 编号、旧提交 SHA、旧分支和记录日期保留。
- 配置解读覆盖 PBD、nonlinear CG、Newton、elliptic/pyramidal、Signorini/convex、rigid coupling、approximate_implicitfast，并提供原生选项、兼容组合、纸上算例、任务诊断和八题答案。
- 沿用 Genesis 1.4.3 官方源码 `216a708e06124595521a9d36a51fae5393fd4ff8`；与本仓既有课程版本一致。`genesis/version.py` 声明该版本；源码 pin、包、构建和历史执行分支仍是不同身份。
- 实验引用冻结在 DexLab `7e4a0774efb4f8b332e866e1495e032b4969c8aa`，保留原报告的 CPU FP64 / Quadrants 1.3.3 边界、无效记录、失败分母和资格限制。本轮没有独立复现实验。
- E7 完整安装与全路线审校继续暂停；本轮只新增来源与静态验证，不启动引擎 import、JIT、仿真、渲染、训练或性能测试。

## 来源复核

直接下载上述 pin 的官方源码阅读，按 `SHA1("blob " + byte_length + NUL + bytes)` 核对 [sources.json](sources.json) 中的 Git blob 身份。复核选项、条件默认、非法组合、Scene 参数、材料导出、PBD stretch、刚体 CG/Newton、积分和 Legacy 耦合；新记录四个源文件：PBD Cloth、版本声明、options 导出与 PBD 材料导出。缓存不提交。

本轮特别核对：

| 结论 | 固定源码定位 |
|---|---|
| Signorini 仅允许 Newton + elliptic + 非兼容；空值按组合解析 | `rigid_solver.py:258–299` |
| elliptic 拒绝 noslip；可微路径另有 gate | `options/solvers.py:655–671`；`rigid_solver.py:752–781` |
| CG 是使用历史方向的非线性方法，提前停止与迭代上限不同 | `constraint/linesearch.py:567–617`；`constraint/solver.py:5502–5564` |
| PBD stretch 的 compliance/h² 与 relaxation；不在该公式中保留 XPBD 乘子历史 | `pbd_solver.py:409–430` |
| Legacy 激活条件与粒子动量反作用，不能等同刚体接触矩阵 | `legacy_coupler.py:43–57,254–309` |
| approximate_implicitfast 的一阶质量修正在动力学入口已参与计算 | `abd/forward_dynamics.py:100–130,353–370` |

对应的完整官方固定链接在正文就近给出。对于 `rigid coupling` 的旧记录，本轮只解释原生路径并提示追溯实际 coupler，没有从当前 API 补写历史身份。

## 静态检查与明确限制

- `git diff --check`：通过。
- `python scripts/check_docs.py`：仓库相对 Markdown 链接通过。
- 对 6 个 `examples/*.py` 和新章节 2 个 Python fenced snippets 执行 `ast.parse`：通过；没有执行片段。
- 新章节的固定 GitHub 源文件 URL 可读取，所有 `#L...` 行号均在实际源文件范围内；实际关键行已人工对照，不把 HTTP 成功当算法验证。
- 本轮读取的 15 个官方文件与来源清单的 Git blob 一致；新增四项另与官方 Git tree 比对。
- 仓库迁移后 9 个现行系列入口及 Issue URL 返回 HTTP 200，两个固定 DexLab 来源文件可读取；旧名称残留按下表逐项解释。

本仓既有 CI 仅包含 diff 和本地文档链接检查。这里的来源、字段与语法结果不证明安装成功、所有组合可运行、任务适用、物理准确或性能提升。原生运行验收仍是后续独立阶段。

## 旧名称保留清单

| 保留位置 | 原样保留内容 | 原因 |
|---|---|---|
| `docs/e3-acceptance.md`、`e4-acceptance.md`、`e5-acceptance.md`、`e6-acceptance.md` 各自第 5 行 | `Sim Atlas` | 2026-10-09 交付记录描述当时的系列名称；保留历史事实，现行入口已更新 |
| `examples/assets/two_joint_arm.urdf:3` | `atlas_two_joint_arm` | 已发布示例的模型身份，名称迁移不改资产语义 |

仓库元数据与本地工作树存储位置不属于产品文档，不因品牌迁移改写历史存储路径。没有对现有 E1–E6 历史验收结果重新署名或扩大完成范围。
