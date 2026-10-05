"""Read installed native libraries sequentially before their mmap-based import.

The import bundle's native modules remain on the executable shared mount.
Warming their original inodes avoids cold, random shared-filesystem page faults;
no packages are replaced or executed from the noexec temporary directory.
"""
import argparse,os,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor


def main():
    p=argparse.ArgumentParser();p.add_argument('import_root',type=Path);args=p.parse_args()
    started=time.monotonic();paths=set()
    for package in args.import_root.iterdir():
        if not package.is_dir():continue
        for current,dirs,files in os.walk(package,followlinks=True):
            dirs[:]=[d for d in dirs if d not in ('include','bin','tests','__pycache__')]
            for name in files:
                if '.so' in name:paths.add((Path(current)/name).resolve())
    def read(path):
        total=0
        with path.open('rb') as stream:
            while block:=stream.read(8*1024*1024):total+=len(block)
        return total
    print('NATIVE_IMPORT_WARM_BEGIN',len(paths),flush=True)
    with ThreadPoolExecutor(max_workers=4) as pool:total=sum(pool.map(read,sorted(paths)))
    print('NATIVE_IMPORT_WARM_READY',dict(files=len(paths),bytes=total,elapsed_s=time.monotonic()-started),flush=True)


if __name__=='__main__':main()
