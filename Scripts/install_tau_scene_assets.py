"""Import the collaborator's PARbar scene and descriptors, preserving asset handles."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

from fit_tau_parbar_supports import fit_supports


def install(bundle, project, manifest):
    sources = [(p, project / p.relative_to(bundle / 'scene'))
               for p in (bundle / 'scene/Assets').rglob('*') if p.is_file()]
    sources += [(p, project / 'Assets/ManualAssets/Descriptors/TauSeasonal' / p.name)
                for p in (bundle / 'descriptors').glob('*.sorghumls')]
    project_file = project / 'test_lsystem_sorghum.eveproj'
    if not project_file.exists():
        sources.append((bundle / 'scene/test_lsystem_sorghum.eveproj', project_file))
    records = []
    for src, dst in sorted(sources):
        payload = src.read_bytes()
        if payload.startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ValueError(f'Bundled asset is a Git LFS pointer: {src}')
        existing = dst.read_bytes() if dst.exists() else None
        if existing is not None and existing != payload and not existing.startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ValueError(f'Existing asset differs; resolve before import: {dst}')
        records.append(dict(source=str(src), destination=str(dst), sha256=hashlib.sha256(payload).hexdigest(),
                            action='unchanged' if existing == payload else 'copy'))
    for (src, dst), record in zip(sorted(sources), records):
        if record['action'] == 'copy':
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(records, indent=2), encoding='utf-8')
    support_report = fit_supports(project)
    manifest.with_name(manifest.stem + '_parbar_supports.json').write_text(
        json.dumps(support_report, indent=2), encoding='utf-8')
    print(f"Imported {sum(r['action'] == 'copy' for r in records)} files; verified {len(records)} files. Manifest: {manifest}")


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--project', type=Path, default=root / 'Resources/DigitalAgricultureProject')
    parser.add_argument('--manifest', type=Path, default=root / 'out/tau_current/asset_import.json')
    args = parser.parse_args()
    install(args.bundle.resolve(), args.project.resolve(), args.manifest.resolve())
