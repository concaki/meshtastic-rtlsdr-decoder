"""Full chain: synthetic .cu8 recording (as rtl_sdr writes it) -> mesh_sdr.py -> JSONL log."""
import json
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def test_synthetic_recording(tmp_path):
    cu8 = tmp_path / "teste.cu8"
    gen = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "gerar_sinal_teste.py"), str(cu8)],
                         capture_output=True, text=True, check=True, cwd=ROOT)
    cmd_line = [ln for ln in gen.stdout.splitlines() if ln.startswith("python mesh_sdr.py")][0]
    # reuse the exact keys printed by the generator
    import shlex
    args = shlex.split(cmd_line)[2:]
    log = tmp_path / "out.jsonl"
    i = args.index("--log")
    args[i + 1] = str(log)
    env = dict(os.environ, PYTHONUTF8="1", COLUMNS="120")
    subprocess.run([sys.executable, os.path.join(ROOT, "mesh_sdr.py"), *args], check=True, cwd=ROOT, env=env,
                   capture_output=True, text=True)
    recs = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    assert len(recs) == 13
    assert all(r["lora"]["crc_ok"] for r in recs)
    decoded = [r for r in recs if r.get("decifragem")]
    assert len(decoded) == 12                    # one packet uses a key we do not have, on purpose
    texts = [r["conteudo"].get("texto") for r in decoded]
    assert "DM secreta para você" in texts       # PKI
    assert "canal com AES-CCM" in texts          # AEAD channel
    assert "sinal bem fraco" in texts            # -10 dB SNR
