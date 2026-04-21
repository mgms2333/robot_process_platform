# 码垛模板示例

## 作用

该目录提供一个最小码垛模板示例，用于后续模板开发参考。

它的目标不是实现完整码垛业务，而是固定以下约定：

- 模板必须独立于平台主体源码
- 模板必须实现 `ITemplate`
- 模板必须遵循 `Plan -> BuildTask` 两阶段结构
- 模板最终必须输出统一的 `Task / ExecutionBlock / Action` 模型
- 模板必须通过 `.so` 导出接口接入 `TemplateManager`

## 当前文件

- `include/palletizing/palletizing_template.h`
  码垛模板头文件

- `src/palletizing_template.cpp`
  码垛模板主流程实现

- `src/palletizing_plugin_entry.cpp`
  插件导出入口

## 当前示例逻辑

当前示例已经固定模板主流程和插件边界，并先对接了
`tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json` 的一版输入解析。

1. `CreateTask`
   接收一段上下文 JSON 字符串

2. `ParseTaskParameters`
   读取 `uuid`、`palletDirection`、`box`、`teachPoint.boxPoint`、`pallet.upperLeft`、`pallet.layer`

3. `Plan`
   负责按箱子生成 `BoxPlan`，并在每个箱子下规划 `pick / scan / transfer / place` 四个工艺块

4. `BuildTask`
   负责把 `ProcessPlan` 展开为统一 `Task / ExecutionBlock / Action`

## 当前参数策略

调用者只向平台传模板名和一段 JSON。
JSON 结构由模板自己定义和解析。当前示例先支持
`tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json` 的核心字段，并基于这些字段按箱子推导取料位、扫码位、过渡位和放料位。

后续其他模板也应沿用这个模式，把模板专属参数尽早收敛为强类型对象。

## 后续扩展方式

后续若新增焊接、上下料、搬运等模板，应仿照该目录新增：

- `plugins/welding/`
- `plugins/loading/`
- `plugins/material_handling/`

每个模板目录内部也应保持 `include/` 与 `src/` 分离。
