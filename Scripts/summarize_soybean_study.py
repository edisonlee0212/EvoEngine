#!/usr/bin/env python3
"""Write a traceable HTML research log from the shared analysis outputs, without new measurements."""
import argparse
import hashlib
import html
import json
import subprocess
from pathlib import Path

import pandas as pd


def summarize(study):
    study = Path(study)
    analysis = study / "analysis"
    facts = json.loads((analysis / "facts.json").read_text(encoding="utf-8"))
    protocol = json.loads((study / "protocol.json").read_text(encoding="utf-8"))
    timing = pd.DataFrame([json.loads(path.read_text(encoding="utf-8")) for path in sorted((study / "timings").glob("*.json"))])
    complete = timing[timing.exit_code == 0].copy()
    finished = timing[timing.exit_code.notna()].copy()
    starts, ends = pd.to_datetime(finished.started_at_utc), pd.to_datetime(finished.completed_at_utc)
    # Parallel generation overlaps. Report both the wall-clock envelope and total work-stage duration.
    timing_summary = {"completed_stages": len(complete), "completed_experiment_cases": sum(f["cases"] for f in facts.values()),
                      "failed_stages": int((finished.exit_code != 0).sum()),
                      "leaf_stage_elapsed_seconds_sum_including_failures": finished.loc[~finished.stage.str.startswith("batch_"), "elapsed_seconds"].sum(),
                      "wall_seconds_first_recorded_stage_to_last_completion": (ends.max() - starts.min()).total_seconds(),
                      "first_stage_started_at_utc": starts.min().isoformat(), "last_stage_completed_at_utc": ends.max().isoformat()}
    (analysis / "timing_summary.json").write_text(json.dumps(timing_summary, indent=2) + "\n", encoding="utf-8")
    chunks = ['<!doctype html><meta charset="utf-8"><title>大豆实验记录</title><style>body{font:17px/1.6 "Microsoft YaHei",Arial;max-width:1180px;margin:48px auto;color:#233d34}table{border-collapse:collapse;font-size:14px;max-width:100%;overflow:auto;display:block}th,td{padding:7px 12px;border:1px solid #ccd5cc;text-align:right}th{background:#e8ede6}img{max-width:100%}code{font-size:14px}h2{margin-top:45px}</style>',
              '<h1>大豆叶形与受光实验记录</h1>',
              '<p>本报告与 notebook、slides 读取同一分析输出。原始 CSV 不修改。只汇总有 completion.json 且通过数值校验的实验批次。PPTX 原生图表的可编辑 Excel 数据保留 12 位有效数字，以满足工作簿精度限制；CSV 保留原始精度。</p>',
              '<h2>实验状态与耗时</h2>', pd.DataFrame([timing_summary]).to_html(index=False),
              finished[["stage", "started_at_utc", "completed_at_utc", "elapsed_seconds", "exit_code"]].to_html(index=False),
              '<p>阶段耗时包括初始化和文件读写，也保留失败尝试的时间。生成库并行进行，阶段耗时之和不等于墙钟时间。墙钟范围从第一次记录的构建阶段开始，包含中途等待批准与修复，不含此前讨论和代码准备。</p>',
              '<h2>生成参数与比较口径</h2>',
              '<p>每类 1024 株，其中 768 株筛选、256 株独立验证。共同生成 seed 2100000–2101023，45 天，三出叶枝最大 25 节，节间最大 0.035 m，小叶原型尺度 U(0.10,0.14) m，叶柄 U(0.12,0.16) m。胖叶 aspect ratio=1.0，瘦叶=0.3，均用于三出叶和单出叶；单出叶尺度仍为默认 0.02 m。其余参数见 protocol.json 与源码快照。</p>',
              '<p>相同输入 seed 不意味着全部几何相同。横向细分数为 8 和 4。Helios 通过下游叶面积调整茎粗。这里比较完整生成植株，没有固定叶面积或茎形。leaf_count 是小叶对象数，不是三出复叶数量。</p>']
    experiment_times = analysis / "experiment_timings.csv"
    if experiment_times.exists():
        chunks += ['<h2>实验批次与调用耗时</h2>', pd.read_csv(experiment_times).to_html(index=False),
                   '<p>illumination_api_seconds 包含探针准备、场景同步与 OptiX 调用，不是独立测得的 GPU kernel 时间。批次总时间还包括初始化、截图、场景保存和退出。</p>']
    geometry = analysis / "geometry_representative/plant_geometry.csv"
    if geometry.exists():
        chunks += ['<h2>示例单株实际几何</h2>', pd.read_csv(geometry).to_html(index=False, float_format=lambda x:f"{x:.5g}"),
                   '<p>Helios leaf-object area 考虑纹理透明度，并可能包括小叶柄。Engine leaf mesh 面积仅对应导出 leaf 组的几何三角形，两者不相等。</p>']
    geometry_library = analysis / "geometry_library/plant_geometry.csv"
    if geometry_library.exists():
        geometry_frame = pd.read_csv(geometry_library)
        chunks += ['<h2>完整植物库几何分布</h2>', geometry_frame.groupby("asset_name")[["leaf_count", "height_m", "leaf_mesh_area_m2", "shoot_mesh_area_m2"]].agg(["mean", "std", "min", "max"]).to_html(float_format=lambda x:f"{x:.5g}")]
    chunks += ['<h2>受光指标</h2>',
               '<p>叶平均 = ΣA(E正+E背)/(2ΣA)；叶合计入射 = ΣA(E正+E背)；全株合计 = 叶、茎、叶柄等合计相加。面积计一次。Field 的土地分母 = ROI 株数 × 行距 × 株距。单株不报告单位土地量。</p>',
               '<p>植株间 CV 使用 ROI 内各株叶平均受光的样本标准差除以算术均值。它衡量田内不均匀性，不是重复实验均值的标准误。</p>',
               '<p>所有光照指标都是相对线性 RGB 经 0.2126R+0.7152G+0.0722B 得到的分数。它们没有 W、PAR 或吸收能量标定，也不是全天积分。合计包含多个表面上的入射贡献，不能直接解释为净截获率。</p>']
    for name, fact in facts.items():
        chunks += [f'<h2>{html.escape(name)}</h2>',
                   f'<p>{fact["cases"]} 个 case，{fact["measured_plants"]} 条单株测量，{fact["unique_generation_seeds"]} 个唯一生成 seed。校验：{fact["reconciliation"]}。</p>']
        metric_rows = [{"metric": key, **value} for key, value in fact["metrics"].items()]
        chunks += [pd.DataFrame(metric_rows).to_html(index=False, float_format=lambda x:f"{x:.5g}")]
        if name != "00_representative_pair" and name != "01_single_plant":
            chunks += ['<p>上表为该批次已测 case 的等权均值，不是最优值，也不是按土地面积合并的单一 Field。各间距及配对 seed 的结果保留在逐 case 表中。</p>']
        if name == "00_representative_pair":
            for number, label in [(0,"胖叶"),(1,"瘦叶")]:
                chunks += [f'<h3>{label}</h3><img src="../raw/{name}/captures/run_{number:04d}.png">']
        if name == "03_field_search" or name.startswith("04_holdout"):
            chunks += ['<h3>同条件下的叶型均值比较</h3>', pd.DataFrame(fact["condition_comparisons"]).to_html(index=False),
                       '<p>这些是逐条件的重复均值方向计数，不是显著性检验。相同间距下土地分母相同，所以单株与单位土地的胖瘦比一致；跨间距排序仍可不同。</p>']
            ranked = pd.read_csv(analysis / name / "ranked_combinations.csv")
            for metric in ["leaf_mean_irradiance", "leaf_incident_per_plant", "leaf_incident_per_ground_area"]:
                columns = ["asset_name", "row_spacing_inches", "column_spacing_inches", "repeats", metric, metric + "_std"]
                chunks += [f'<h3>{metric} 的前三组（每类）</h3>', ranked.sort_values(metric, ascending=False).groupby("asset_name").head(3)[columns].to_html(index=False)]
            chunks += ['<p>这些排序是测试网格内的探索性比较。配对差异与逐点区间在 paired_summary.csv，区间未作多重比较校正，不用于宣称显著性。</p>']
        if name == "03_field_search" and (study / "figures/field/capture_manifest.json").exists():
            chunks += ['<h3>相同间距下的完整 Field</h3>']
            for number, label in [(245, "胖叶"), (740, "瘦叶")]:
                chunks += [f'<p>{label}，行距 22.5 inch，株距 3 inch，subset seed 101</p><img src="../figures/field/run_{number:04d}.png">']
        for image in sorted((analysis / name).glob("*.png")):
            chunks += [f'<img src="{name}/{image.name}">']
    convergence = analysis / "sample_convergence.csv"
    if convergence.exists():
        frame = pd.read_csv(convergence)
        chunks += ['<h2>采样数值敏感性</h2>', frame.groupby(["samples","metric"]).relative_difference_from_256.agg(["mean","min","max"]).to_html(),
                   '<p>256 samples 是数值参考，不是真值；这里没有隔离所有可能的建模误差。</p>']
    events = study / "provenance/coordination_events.json"
    for filename, title, explanation in [
        ("holdout_validation.csv", "独立库对训练最优的检验", "固定训练阶段选中的最优间距，报告它在独立库候选集内的排名及相对最佳候选的差距；不是用验证库重新做全网格搜索。"),
        ("buffer_sensitivity.csv", "外围缓冲区敏感性", "比较 8×20 与 12×30 Field 的中央 40 株；中央组成不同，不是只改变外围植物的因果对照。"),
        ("edge_control.csv", "ROI 接收器与边缘对照", "同一完整 Field 测全部 160 株，再提取相同中央 40 株，与原 ROI 测量对账。外围与中央差异为描述性比较。"),
    ]:
        if (analysis / filename).exists():
            chunks += [f'<h2>{title}</h2><p>{explanation}</p>', pd.read_csv(analysis / filename).to_html(index=False)]
    if events.exists():
        chunks += ['<h2>运行记录与修正</h2>', pd.DataFrame(json.loads(events.read_text(encoding="utf-8"))).to_html(index=False)]
    buffer_file = analysis / "buffer_sensitivity.csv"
    if buffer_file.exists():
        changes = pd.read_csv(buffer_file).groupby(["asset_name", "row_spacing_inches", "column_spacing_inches"]).leaf_mean_irradiance_relative_change.mean()
        maximum = changes.abs().max()
        chunks += [f'<h2>稳健性结论的边界</h2><p>扩大 Field 且中央组成变化时，逐间距对 3 个子集相对变化取均值后，叶面平均受光的最大绝对变化为 {maximum:.1%}。密植结果存在明显敏感性，不能声称已验证无限 Field 收敛或对 Field 大小稳健的最优间距。独立植物库中的候选排名稳定，不会消除这一限制。这里只完成原定敏感性试验，没有自动扩展搜索或重跑全网格。</p>']
    recovery = study / "provenance/recovery_plan.json"
    if recovery.exists():
        chunks += ['<h2>CUDA 失败与经批准的续跑</h2>',
                   '<p>主搜索原尝试完成 890/990 个 case 后，在创建 CUDA surface 时发生内存错误，退出码 1。原始 CSV 与失败日志保留不变。修复只改变资源生命周期，未改变测光公式、模型或种子。先做两个参考 case 对照，再只执行剩余 100 个 case；完整数据在 assembled/03_field_search，参考重测不计作统计重复。</p>',
                   f'<pre>{html.escape(json.dumps(json.loads(recovery.read_text(encoding="utf-8")), indent=2, ensure_ascii=False))}</pre>']
        reference = study / "provenance/repair_reference_check.json"
        if reference.exists():
            chunks += [f'<pre>{html.escape(reference.read_text(encoding="utf-8"))}</pre>']
    chunks += ['<h2>统计解释检查</h2><ol>',
               '<li>分组反转：分别报告叶型、间距和目标，不以全网格均值替代逐条件比较。</li>',
               '<li>生态谬误：Field 平均不代表每株都更高。</li>',
               '<li>样本选择：种子来自生成器，不代表真实品种总体。</li>',
               '<li>碰撞变量：未通过受光或叶面积筛掉植株。</li>',
               '<li>基准率：不适用于诊断/分类指标。</li>',
               '<li>均值回归：训练最优候选使用独立植物库重新评估。</li>',
               '<li>幸存者偏差：失败阶段保留日志，不静默重试或删除失败数据。</li>',
               '<li>多重搜索：报告完整网格，不宣称搜索最优具有确认性显著性。</li>',
               '<li>分析自由度：protocol.json 保存本次设定，结果为探索性。</li>',
               '<li>因果外推：只描述给定模型和环境下的结果，不推断真实作物产量。</li>',
               '<li>反向因果：静态仿真无生长反馈，不能用于推断反馈机制。</li></ol>',
               '<h2>限制与可复现文件</h2><p>接收面积采用几何三角形，未做纹理 alpha 的分数面积校正。太阳固定正上方。静态植株直接排布，密植时可能相互穿插，没有模拟碰撞避让或生长适应。不同 field 大小的中央植株组成不同，因此边界敏感性包含有限样本的组成差异。新旧 total_flux 口径不能混合。</p>',
               '<p>日志仍包含 Vulkan storage-buffer descriptor 超过 maxStorageBufferRange 的验证错误，未在本任务中修复。当前 OptiX 路径在 RayTracerLayer.cpp 读取 CPU mesh，并由 OptiXRayTracer.cu 单独上传顶点和三角形；不依赖上述 Vulkan geometry descriptor。这一源码检查和数值对账不等于整个渲染器通过验证，也不能替代物理标定。</p>',
               '<p>参数在 ../configs，原始测量在 ../raw，进程输出在 ../logs，逐阶段计时在 ../timings，源码与版本快照在 ../provenance。截图来自安装目录中的 PyEvoEngine.cp312-win_amd64.pyd 调用 OptiX，不是外部绘制的模型图。</p>']
    scenes = []
    for config_file in sorted((study / "configs").glob("*.json")):
        config = json.loads(config_file.read_text(encoding="utf-8"))
        prefix = config.get("save_scene_prefix")
        if not prefix:
            continue
        for run_id in config.get("capture_run_ids", []):
            scene = Path(config["project"]).parent / "Assets" / f"{prefix}_{run_id:04d}.evescene"
            if scene.exists() and all(entry["path"] != str(scene) for entry in scenes):
                scenes.append({"experiment": config_file.stem, "run_id": run_id, "path": str(scene), "bytes": scene.stat().st_size})
    (study / "scene_index.json").write_text(json.dumps(scenes, indent=2) + "\n", encoding="utf-8")
    chunks += ['<h2>场景快照索引</h2>', pd.DataFrame(scenes).to_html(index=False),
               '<p>完整场景会内嵌大量网格数据，部分 Field 快照约 3 GB。未逐份重新载入验证。查看截图、CSV 和参数不需要加载这些文件；通常可直接从模型库和配置重新生成 Field。</p>',
               '<h2>安装与运行命令</h2>']
    for _, row in complete[complete.stage.str.contains("install")].iterrows():
        chunks += [f'<p>{html.escape(row["stage"])}：exit code 0</p><pre>{html.escape(subprocess.list2cmdline(row["command"]))}</pre>']
    (analysis / "experiment_report.html").write_text("\n".join(chunks), encoding="utf-8")
    checksums = {}
    for filename in sorted((study / "raw").rglob("*")):
        if filename.is_file() and filename.suffix in {".csv", ".json", ".png"}:
            with filename.open("rb") as stream:
                checksums[filename.relative_to(study).as_posix()] = hashlib.file_digest(stream, "sha256").hexdigest()
    (study / "provenance/raw_checksums.json").write_text(json.dumps(checksums, indent=2) + "\n", encoding="utf-8")
    return timing_summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, required=True)
    print(json.dumps(summarize(parser.parse_args().study), indent=2))
