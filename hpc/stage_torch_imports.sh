# Source once before launching independent Isaac processes in an allocation.
# Build the bundle with hpc/pack_torch_imports.py on the login node first.
MP_TORCH_ARCHIVE=${MULTIPATH_TORCH_ARCHIVE:-$ROOT/outputs/runtime_cache/torch_imports.tar.gz}
if [[ ! -f "$MP_TORCH_ARCHIVE" ]]; then
 echo "Missing Torch import bundle: $MP_TORCH_ARCHIVE; run hpc/pack_torch_imports.py before submission" >&2
 exit 1
fi
# /tmp is node-local; a shared TMPDIR would reproduce the original import stall.
MULTIPATH_IMPORT_ROOT=$(mktemp -d "/tmp/multipath-imports-${PBS_JOBID:-local}.XXXXXX")
trap 'rm -rf -- "$MULTIPATH_IMPORT_ROOT"' EXIT
echo "TORCH_IMPORT_STAGE_BEGIN archive=$MP_TORCH_ARCHIVE destination=$MULTIPATH_IMPORT_ROOT"
timeout 90 tar -xzf "$MP_TORCH_ARCHIVE" -C "$MULTIPATH_IMPORT_ROOT"
echo "TORCH_IMPORT_STAGE_READY destination=$MULTIPATH_IMPORT_ROOT"
# Cache warming is an optimization, not a correctness gate. A cold node's
# shared-file read must not spend minutes or abort before the real imports.
if timeout -k 5 "${MULTIPATH_NATIVE_WARM_SECONDS:-20}" "$RUNTIME/envs/retarget/bin/python" "$ROOT/hpc/warm_native_imports.py" "$MULTIPATH_IMPORT_ROOT"; then
 :
else
 echo "NATIVE_IMPORT_WARM_INCOMPLETE proceeding_to_watched_imports"
fi
