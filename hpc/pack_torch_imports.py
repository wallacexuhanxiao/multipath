#!/usr/bin/env python3
"""Bundle Torch import resources once on the login node, without importing Torch.

Keep large native libraries and compilation headers in the installed environment;
only small import resources are unpacked on each allocated node. Rebuild after an
environment update. The archive preserves the installed package, not another
Torch version supplied by the container.
"""
import argparse
import json
import os
from pathlib import Path
import tarfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--torch-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--extra-package',action='append',type=Path,default=[],help='Installed scientific package import resources (e.g. numpy/scipy).')
    args = parser.parse_args()
    source = args.torch_dir.resolve(strict=True)
    if source.name != 'torch' or not (source / '__init__.py').is_file():
        parser.error('--torch-dir must be the installed Torch package directory')
    started = time.monotonic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temp = args.output.with_suffix(args.output.suffix + '.tmp')
    shared = {'lib', 'include', 'bin'}
    count = 0
    with tarfile.open(temp, 'w:gz', compresslevel=1) as archive:
        for package in [source]+[p.resolve(strict=True) for p in args.extra_package]:
            package_shared=shared if package==source else set()
            if not (package/'__init__.py').is_file():parser.error(f'Not an installed package: {package}')
            for current, dirs, files in os.walk(package):
                rel = Path(current).relative_to(package)
                dirs[:] = sorted(d for d in dirs if d != '__pycache__'
                                 and not (package!=source and d=='tests')
                                 and not (rel == Path('.') and d in package_shared))
                archive.add(current, arcname=str(Path(package.name) / rel), recursive=False)
                for name in sorted(files):
                    if not name.endswith(('.pyc', '.pyo')):
                        member = str(Path(package.name) / rel / name)
                        if '.so' in name:
                            # Hopper /tmp is noexec: load native extensions from the
                            # original executable mount, just like torch/lib below.
                            link = tarfile.TarInfo(member)
                            link.type = tarfile.SYMTYPE
                            link.linkname = str(Path(current) / name)
                            archive.addfile(link)
                        else:
                            archive.add(Path(current) / name, arcname=member, recursive=False)
                        count += 1
            for name in sorted(package_shared):
                if (package / name).exists():
                    link = tarfile.TarInfo(package.name+'/' + name)
                    link.type = tarfile.SYMTYPE
                    link.linkname = str(package / name)
                    archive.addfile(link)
            sibling=package.parent/(package.name+'.libs')
            if sibling.exists():
                link=tarfile.TarInfo(sibling.name);link.type=tarfile.SYMTYPE;link.linkname=str(sibling)
                archive.addfile(link)
    temp.replace(args.output)
    report = dict(source=str(source), archive=str(args.output.resolve()),
                  import_files=count, compressed_bytes=args.output.stat().st_size,
                  shared_directories=sorted(shared), extra_packages=[str(p) for p in args.extra_package], elapsed_s=time.monotonic()-started)
    args.output.with_suffix(args.output.suffix + '.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
