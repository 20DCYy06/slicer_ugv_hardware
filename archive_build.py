from pathlib import Path
import sys,hashlib,json,shutil
root=Path(__file__).resolve().parent;version=root/'Firmware'/sys.argv[1]
assert root in version.resolve().parents
b=version/'bin';b.mkdir(exist_ok=True)
for src,name in [('c30d_body.bin','firmware.bin'),('c30d_body.hex','firmware.hex'),('c30d_body.elf','firmware.elf'),('c30d_body.map','firmware.map'),('generated.h','generated.h'),('manifest.json','manifest.json')]:shutil.copy2(version/'src/build'/src,b/name)
records={}
for p in sorted(version.rglob('*')):
    if p.is_file() and 'build' not in p.relative_to(version).parts and p.name!='source_manifest.json':records[str(p.relative_to(version))]=hashlib.sha256(p.read_bytes()).hexdigest()
(version/'source_manifest.json').write_text(json.dumps({'version':sys.argv[1],'files':records},indent=2))
print('ARCHIVED',version,'BIN SHA256',records['bin/firmware.bin'])
