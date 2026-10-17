# MovementSync

<!-- 徽章 -->
<p>
  <a href="https://github.com/huangdihd/MovementSync/releases" target="_blank">
    <img src="https://img.shields.io/github/v/release/huangdihd/MovementSync?style=for-the-badge&label=最新版本&color=brightgreen" alt="Latest Release">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/issues" target="_blank">
    <img src="https://img.shields.io/github/issues/huangdihd/MovementSync?style=for-the-badge&label=问题&color=yellow" alt="Issues">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/blob/master/LICENSE" target="_blank">
    <img src="https://img.shields.io/github/license/huangdihd/MovementSync?style=for-the-badge&label=许可&color=blue" alt="License">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/stargazers" target="_blank">
    <img src="https://img.shields.io/github/stars/huangdihd/MovementSync?style=for-the-badge&label=收藏&color=ff69b4" alt="Stars">
  </a>
  <a href="https://jitpack.io/#huangdihd/MovementSync" target="_blank">
    <img src="https://img.shields.io/jitpack/version/com.github.huangdihd/MovementSync?style=for-the-badge&label=JitPack&color=b22222" alt="JitPack">
  </a>
</p>

---

> 一个用于处理 xinbot 机器人移动和地形的插件。  
简体中文 / [English](README.md)

---

## 快速开始

1. **下载**  
   从 [Releases](https://github.com/huangdihd/MovementSync/releases) 获取最新版本：  
   `MovementSync-[version].jar`

2. **复制** 到你的插件目录。

3. **启动** [xinbot](https://github.com/huangdihd/xinbot)。

---

## 命令

| 命令 | 描述 |
|-----------|-------------|
| `whereami` | 显示当前位置和旋转 |
| `jump` | 执行一次跳跃 |
| `getblockat` | 获取指定坐标的方块信息 |
| `walk` | 向指定方向行走一段时间 |
| `lookat` | 看向指定坐标 |
| `movement` | 控制运动控制器 (暂停/恢复/取消) |
| `entities` | 列出世界中的实体 |
| `interactblock` | 与方块交互 |
| `interactentity` | 与实体交互 |
| `inventory` | 显示背包信息 |
| `container` | 显示当前容器内容 |
| `ispassable` | 检查方块是否可通行 |
| `lookatentity` | 通过 ID 看向实体 |
| `goto` | 寻路至指定坐标;目标较远且有鞘翅和烟花时会自动飞过去(`goto <x> <y> <z> walk` 强制步行) |
| `elytra` | 鞘翅飞行:`start`、`boost`、`status`、`fly <x> <z> [y]` |
| `follow` | 通过 ID 跟随实体 |
| `ride` | 骑乘实体或下车 |
| `cansee` | 检查从机器人到头部的视线是否可见某点 |
| `stopmove` | 停止所有运动并清除目标 |
| `debug` | 打印当前调试状态 |

---

## 有屋顶维度的飞行

维度注册表的 `has_ceiling` 决定是否应用屋顶策略；`min_y` / `height` 是世界边界，`logical_height` 不是屋顶高度。屋顶面由已加载列的实际最高碰撞面确定，不写死基岩高度。

- 已在屋顶上时沿用地形上方巡航；静态目标上方有屋顶碰撞时，在目标水平位置附近选择安全屋顶落点。落地后提示“目标在屋顶下,无法到达”，结束 goto 并清除恢复目标。
- 屋顶下 goto 走路，不自动上屋顶或下钻。危险下落仅在现有有界滑翔预演与恢复检查证明安全时展开救援；否则保持原有下落处理。本阶段不提供地下通道飞行。
- 所有维度的标准飞法规则拒绝穿过岩浆、火及灵魂火的轨迹；落地排除热伤害支撑（包括岩浆块、点燃的营火）及站立身体附近的岩浆。使用保守整格接触检查，不模拟持续燃烧和伤害冷却。

---

## 社区与支持

- **问题**：报告 Bug 或提出功能请求
- **讨论**：分享想法、代码片段或展示你的插件

如果你喜欢 MovementSync，别忘了给它一个 ⭐ 哦！

---

## 许可

GPL-3.0-or-later — 查看 [LICENSE](LICENSE) 获取完整文本。

- 你可以使用、修改和分发此插件。
- 如果分发修改版本，必须公开源码并保持相同许可。
- 在再分发时包含版权和许可声明。

由 huangdihd ❤️ 制作
