# 读懂 DexLab 的 Genesis 配置

**建议先修：**先沿 [Physics-Based Simulation 中文伴读路线](https://github.com/huangkiki/sim-learning/blob/main/docs/physics-based-simulation.md)建立离散化、优化、接触和摩擦基础，再读本引擎的具体配置。

[课程路线](curriculum.md) · [Sim Learning 共同基础](https://github.com/huangkiki/sim-learning) · [原生接触专题](contact-solvers-forces.md) · [多物理专题](extensions-boundaries.md) · [来源与验收](learning-migration-acceptance.md)

面向有 VLA、WAM、RL 经验的工程师及硕博：你已经会组织 rollout、损失或策略，现在要判断训练所用的状态转移究竟由什么模型产生。读完应能拆解一条配置、识别无效组合，并为夹持滑落或布料拉伸提出可以区分原因的诊断步骤。先修是向量、梯度、牛顿第二定律；完整推导和实现细节链接到现有课程。

**版本边界：** DexLab 登记 Genesis 1.4.3；本仓也读取 1.4.3，固定源码为 `216a708e06124595521a9d36a51fae5393fd4ff8`，本轮没有升级版本。下文的字段与限制只适用于该 pin。DexLab 斜面报告另记录 Quadrants 1.3.3、CPU FP64；包版本相同不意味着所有历史运行都有相同构建或配置。本文做来源核对、链接和语法检查，没有 import、JIT、原生步进或新实验；E7 仍暂停。[本仓版本](versions.md) · [上游版本声明][version] · [冻结实验报告][dexlab-report]

**English summary.** This Chinese lesson is for VLA/WAM/RL engineers and graduate researchers. Genesis 1.4.3 uses separate choices for deformable PBD, nonlinear rigid constraint optimization, friction cones, contact resolution, inter-solver coupling and integration. The lesson maps each DexLab label to native APIs and pinned implementation, lists valid rigid combinations, and gives configuration diagnosis and exercises. Existing advanced lessons provide derivations. Source and syntax checks are complete; native execution, physics validation and the paused E7 installation review are separate work. Historical DexLab results are cited with their original conditions, not independently reproduced here.

## 1. 把一条配置读成一句话

假设读到 `Genesis / Newton / elliptic / signorini / approximate_implicitfast`。应解释为：“在指定 Genesis 实现中，用 Newton 求解刚体约束问题，以椭圆锥的坐标组织摩擦，并按该实现的 Signorini 路径处理法向载荷与摩擦界，再按指定积分近似推进时间。”这句话还缺资产、材料、步长、精度、预算、耦合器等实验条件。

| DexLab 名称 | 所处层 | 本版本的原生入口 | 读完应知道什么 |
|---|---|---|---|
| `PBD` | 柔性实现 | `gs.materials.PBD.Cloth`、`gs.options.PBDOptions` | 粒子位置约束，不是刚体 Newton 的别名 |
| `CG (nonlinear)` | 刚体约束数值求解 | `RigidOptions.constraint_solver=gs.constraint_solver.CG` | 直接迭代非线性目标，不是 Newton 内部的线性 CG |
| `Newton` | 刚体约束数值求解 | `gs.constraint_solver.Newton` | 根据梯度与曲率求方向，再做线搜索 |
| `elliptic` / `pyramidal` | 摩擦锥表示 | `RigidOptions.friction_cone` | 改变可行摩擦集合与约束装配 |
| `Signorini` / `convex` | 接触模型选项 | `RigidOptions.contact_resolution`；枚举写作 `gs.contact_resolution.signorini` / `.convex` | 决定法向与摩擦如何耦合，不能从锥名称单独推定 |
| `rigid coupling` | 跨 solver 相互作用路径 | 例如 `LegacyCouplerOptions(rigid_pbd=True)` 与 `Rigid(needs_coup=True, ...)` | 须继续查具体 coupler 与两侧活动 solver；标签本身不完整 |
| `approximate_implicitfast` | 时间积分配置 | `RigidOptions.integrator=gs.integrator.approximate_implicitfast` | 是积分近似，不能当新摩擦模型或无条件稳定性保证 |

表中入口由[刚体选项][options]、[PBD 选项][pbd-options]、[PBD 布料材料][cloth]、[耦合选项][coupler-options]与[材料分派][dispatch]核对。DexLab 的[分类词表][dexlab-taxonomy]汇总出现过的名字，不能把所有列相乘生成“已支持配置”。

对于 RL，以上选择进入离散转移 $s_{t+1}=F_h(s_t,u_t;\theta)$。改变摩擦锥、接触模型或步长，会改变 $F_h$；因此策略回报的差异不能直接归因于“某个 solver 更好”。

## 2. 摩擦锥与接触模型分别回答什么

想象方块放在斜面上。重力在斜面方向拉它下滑，法向支撑使它不继续沉入地面，摩擦抵抗相对滑动。只考虑各向同性滑动摩擦，不含扭转/滚动时，椭圆锥使用

$$
f_n\geq0,\qquad\sqrt{f_{t1}^2+f_{t2}^2}\leq\mu f_n.
$$

固定 $f_n$ 后，切向截面是圆盘；本版本的 pyramidal 对应 $|f_{t1}|+|f_{t2}|\leq\mu f_n$ 的菱形截面。例如 $\mu f_n=1\,\mathrm N$，沿单个切向轴可以到 1 N，而菱形沿 45° 方向的合力上限为 $1/\sqrt2\,\mathrm N$。这是几何算例，不是仿真结果。默认滑动接触的 pyramidal 装成四条混合行，elliptic 装成法向加两条切向三行；不能将切换锥视作只改变迭代速度。[方向与装配][cone-rows]

接触模型再决定这些力如何共同产生。理想硬接触可用间隙 $g\geq0$、支撑力 $f_n\geq0$、$g f_n=0$ 建立直觉：分离时无支撑，接触时允许非零支撑。但 Genesis 的 `signorini` 仍含软参考加速度和正则化，**不承诺每步严格零穿透**。

- `convex` 的椭圆接触在同一个锥块中处理法向与摩擦，切向需求可能影响法向解。
- `signorini` 将当前法向载荷锁存为 $\ell$，摩擦块本轮以 $\mu\ell$ 为半径，下一轮重新更新；法向载荷与摩擦界通过逐次逼近协调。有限迭代的锁存值可能还未等于最终法向力。

来源是[接触块公式][cone-model]及[载荷重锁存][relatch]，详解见 [E3 接触专题](contact-solvers-forces.md)。模型误差、接触柔顺和求解残差是不同问题：收紧容差不会自动把一个正则化模型变成理想硬接触。

## 3. CG 与 Newton：决定怎样求当前子步的解

用 $a$ 表示广义加速度，$a_0$ 表示无约束加速度。固定该子步的质量、Jacobian 和正则参数，convex 路径可用下式理解：

$$
\Phi(a)=\tfrac12(a-a_0)^T M(a-a_0)+\phi(Ja-a_{ref}).
$$

第一项惩罚偏离自由动力学，第二项表达约束代价。这里是加速度空间的数值目标，不是“穿透深度平方”，也不是系统的势能。Signorini 的摩擦半径跨轮更新，不能给整个过程套上同一个固定凸目标的保证。[目标与梯度][objective]

Newton 用 $H p=-\nabla\Phi$ 得到搜索方向，本实现维护按岛组织的 Hessian/Cholesky；接触块带来额外曲率。非线性 CG 用预条件梯度和历史方向构造下一方向，代码使用 Hager–Zhang 系数，随后线搜索。两者都在求当前约束问题，不是积分器的两个名称。[迭代实现][iteration] · [CG 与停止判据][exit]

`iterations` 是最多迭代多少次，`tolerance` 参与按惯量尺度缩放的终止判据，`ls_iterations` 是线搜索预算。它们都不是以米表示的最大穿透保证。已满足某个提前停止条件时，将上限从 100 提到 1000 可能完全不改变结果；定位问题时必须先区分提前停止与预算耗尽。

## 4. 原生配置：有效组合先于参数调优

以下表格限定原生刚体约束路径、`requires_grad=False`、`enable_mujoco_compatibility=False`；不表示已经验证各配置适用于任意任务。

| 约束方法 | 锥 | `convex` | `signorini` |
|---|---|---|---|
| Newton | elliptic | 允许 | 允许 |
| Newton | pyramidal | 允许 | 拒绝 |
| 非线性 CG | elliptic | 允许 | 拒绝 |
| 非线性 CG | pyramidal | 允许 | 拒绝 |

初始化源码要求 Signorini 同时满足 Newton、elliptic、非 MuJoCo 兼容；`contact_resolution=None` 仅在这些条件成立时解析为 Signorini，否则为 convex。`elliptic + noslip_iterations>0` 被拒绝；可微刚体也拒绝 elliptic。SAP/IPC coupler 又有独立路径，不能套用这张表解释其实际约束求解。[组合解析][resolve] · [参数校验][validation] · [可微限制][grad-gates]

下面是**未执行的配置阅读片段**。数值用于说明字段，不是任务推荐参数；仅创建选项，未初始化引擎、构建场景或推进时间。

```python
import genesis as gs

newton_signorini = gs.options.RigidOptions(
    constraint_solver=gs.constraint_solver.Newton,
    friction_cone=gs.friction_cone.elliptic,
    contact_resolution=gs.contact_resolution.signorini,
    integrator=gs.integrator.approximate_implicitfast,
    enable_mujoco_compatibility=False,
    iterations=100,
    tolerance=1e-8,
    noslip_iterations=0,
    impratio=100.0,
)

cg_convex = gs.options.RigidOptions(
    constraint_solver=gs.constraint_solver.CG,
    friction_cone=gs.friction_cone.elliptic,
    contact_resolution=gs.contact_resolution.convex,
    integrator=gs.integrator.approximate_implicitfast,
    enable_mujoco_compatibility=False,
    iterations=100,
    tolerance=1e-8,
    noslip_iterations=0,
    impratio=100.0,
)
```

若目的是隔离 CG/Newton 的求解方法差异，应令**两者都采用 convex**，再固定其余条件；上面的 Newton/Signorini 与 CG/convex 同时改变两个因素，只能比较完整候选配置。相同迭代上限也不等于相同计算成本。

## 5. PBD、刚体耦合与积分为什么另占一层

把场景换成夹爪拉布料。PBD 将布料离散为粒子和连接约束，预测位置后逐轮修正长度、弯曲等误差。两个粒子的伸长约束为 $C=\|x_1-x_2\|-\ell_0$；本版本 stretch 更新的一个粒子修正为

$$
\Delta x_1=-\frac{w_1 C}{w_1+w_2+\alpha/h^2}\,\hat n\,r,
$$

其中 $w_i$ 是可运动粒子的逆质量，$\alpha$ 是伸长 compliance，$h$ 是子步，$r$ 是 relaxation。该 kernel 没有在这条更新中保留跨迭代的 XPBD 乘子历史，不能仅因出现 compliance 就把它认作完整标准 XPBD。迭代数、步长、compliance、relaxation 和网格都会影响离散响应。[PBD stretch][pbd-stretch] · [E6 多物理推导](extensions-boundaries.md)

夹爪与布料相互作用还需要跨 solver 耦合。例如下面的原生选项表示 Legacy 的 rigid–PBD 开关和双方材料；仍是未执行的字段阅读片段：

```python
pbd_options = gs.options.PBDOptions(max_stretch_solver_iterations=4)
coupler_options = gs.options.LegacyCouplerOptions(rigid_pbd=True)
cloth_material = gs.materials.PBD.Cloth(stretch_compliance=1e-7)
gripper_material = gs.materials.Rigid(
    needs_coup=True,
    friction=0.5,
    coup_friction=0.5,
    coup_restitution=0.0,
)
```

这几项分别交给 Scene 的 `pbd_options` / `coupler_options` 和相应实体的 `material`。仅写 `rigid_pbd=True` 不会凭空生成布料或刚体：build 还检查两侧 solver 是否活动，几何必须参加耦合。[Scene 参数][scene] · [活动门槛][coupling-gate] · [材料][material]

`friction` 用于原生刚体接触，`coup_friction` 用于耦合响应。Legacy 的粒子碰撞路径会改变相对速度，并把粒子动量变化以 $-\Delta p/h$ 反馈到刚体；这不是刚体 contact 矩阵里的同一条摩擦律，也不能由 `get_contacts()` 的刚体点力读数代表全部耦合力。[Legacy 响应][coupling-response]。已有 `rigid coupling` 标签仍须追报告中的具体实现，不能因为本文举 Legacy 例子就补填历史身份。

最后，`approximate_implicitfast` 决定时间离散：它在前向动力学的质量矩阵处理中纳入一阶隐式修正，并沿用到约束计算；这与单纯增大 Newton 迭代数不同。子步 $h=\Delta t/\mathrm{substeps}$ 会改变该离散模型，也进入上述 PBD compliance 缩放。不能把名称解释成完全隐式积分、无条件稳定，或与 MuJoCo `implicitfast` 数学等价。[动力学入口][integration] · [一阶修正][integration-correction] · [E1 时间基础](modeling-state-time.md)

## 6. 从任务现象回到可核对的原因

| 任务现象 | 优先核对 | 后续有运行授权时，怎样区分原因 |
|---|---|---|
| RL 策略在另一套 Genesis 设置下滑落 | cone、resolution、耦合路径、有效摩擦、控制频率 | 固定控制输入回放，先区分物理配置变化与策略分布变化 |
| nominal 摩擦设零仍有切向阻力 | 输入、接触组合后的有效值、读取阶段 | 读逐接触有效摩擦；不能只看 setter 成功或参数文件 |
| 增加 CG 上限后轨迹不变 | 是否提前停止、梯度、残差，而非只看错误位 | 在相同前缀比较改进量、停止阈值和力–动量一致性 |
| Newton/elliptic 换成 CG 后结果改变 | `contact_resolution=None` 的条件默认 | 显式固定 convex 再比较，否则同时改了接触模型 |
| 布料随夹爪动，但反作用不可信 | 实体分派、coupler、needs_coup、附着与动力学响应 | 分别查布料形变与刚体反作用；视觉跟随不能替代力闭环 |
| 提高刚体 `iterations` 后布料仍伸长 | PBD 预算、compliance、子步和网格 | 查 `PBDOptions` 的实际消费者，不能拿刚体预算给布料调刚度 |

这些是诊断方案，不是本轮新测量。已有冻结案例可以练习读证据：[DexLab 斜面报告][dexlab-report]区分五种合法组合和三种被 API 拒绝的 Signorini 组合，并保留名义零摩擦失败与七条 CG 无效记录。其有效摩擦下限、梯度/动量一致性及提前停止诊断均有单独记录。不要将无效记录当成物理任务失败，也不要用后来诊断成功覆盖原失败分母；这里只引用该报告，没有重跑。对于 VLA/WAM 数据集，同样应把物理配置、有效读回、时序和有效性与轨迹一起保存，学习接口见 [E5](batch-learning-data.md)。

## 7. 阅读练习与答案

1. **“PBD 与 Newton 哪个更适合训练？”缺少什么？** 答：先确定对象及问题。PBD 在这里处理柔性粒子位置约束，Newton 是刚体约束问题的数值方法；一个夹爪–布料场景可以同时使用两者，还需明确耦合。
2. **只知道 elliptic 能推断 Signorini 吗？** 答：不能。CG/elliptic 走 convex；Newton/elliptic 也可显式选 convex，兼容开关会改变资格和默认解析。
3. **要比较刚体 CG/Newton 的算法影响，应该给二者配哪个接触模型？** 答：在这张兼容表的范围内选共同支持的 convex，固定 cone、步长、材料、精度和其余条件，再单独记录计算预算；仍不能仅由任务分数证明算法归因。
4. **$\mu f_n=2\,\mathrm N$，滑动方向沿切向 45° 时，两种截面的合力界是多少？** 答：elliptic 是 2 N；本例 pyramidal 是 $\sqrt2$ N。只限各向同性滑动、无扭转/滚动的理想集合示意。
5. **布料太软，把 `RigidOptions.iterations` 翻倍是否直接增加 PBD stretch 迭代？** 答：不会。应核对 `PBDOptions.max_stretch_solver_iterations`、材料 compliance、relaxation、网格与 h 的实际值；调整预算也不能代替材料标定。
6. **报告里 CG 达到某个停止条件，却没通过力–动量一致性，能说代码没报错所以轨迹有效吗？** 答：不能。数值停止、遥测完整、物理一致性与任务表现分别判断，保留无效记录及其原始阈值。
7. **保持模型不变时为什么 1000 次上限可能与 100 次一样？** 答：如果原过程早已因改进量或其他条件停止，更高上限不会触发额外迭代。需要读停止原因，而不是只看配置数字。
8. **1.4.3 与本课相同，能补填一份旧记录没有写出的 solver 吗？** 答：不能。相同包版本不能补齐构建、补丁、运行分支或默认解析证据；保留未知，追实际 manifest 或遥测。

下一步：用 [E3](contact-solvers-forces.md) 追到目标、梯度、停止和力读回；用 [E6](extensions-boundaries.md) 查 PBD、Legacy/SAP/IPC 和可微边界。跨引擎共同概念回到 [Sim Learning](https://github.com/huangkiki/sim-learning)，训练与数据流程接 [Robot Learning](https://github.com/huangkiki/robot-learning)。

[dexlab-taxonomy]: https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/solver-taxonomy.zh-CN.md
[dexlab-report]: https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/genesis-incline-results.zh-CN.md
[options]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/options/solvers.py#L592-L635
[pbd-options]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/options/solvers.py#L804-L847
[cloth]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/materials/PBD/cloth.py#L11-L43
[coupler-options]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/options/solvers.py#L89-L123
[dispatch]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/simulator.py#L115-L153
[cone-rows]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/solver.py#L605-L840
[cone-model]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/solver.py#L4149-L4376
[relatch]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/solver.py#L4661-L4740
[objective]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/solver.py#L4816-L5172
[iteration]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/solver.py#L5502-L5564
[exit]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/constraint/linesearch.py#L567-L617
[resolve]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/rigid_solver.py#L258-L299
[validation]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/options/solvers.py#L655-L671
[grad-gates]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/rigid_solver.py#L752-L781
[pbd-stretch]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/pbd_solver.py#L409-L430
[scene]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/scene.py#L143-L210
[coupling-gate]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/couplers/legacy_coupler.py#L43-L57
[material]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/materials/rigid.py#L25-L115
[coupling-response]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/couplers/legacy_coupler.py#L254-L309
[integration]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/abd/forward_dynamics.py#L100-L130
[version]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/version.py#L1
[integration-correction]: https://github.com/Genesis-Embodied-AI/genesis-world/blob/216a708e06124595521a9d36a51fae5393fd4ff8/genesis/engine/solvers/rigid/abd/forward_dynamics.py#L353-L370
