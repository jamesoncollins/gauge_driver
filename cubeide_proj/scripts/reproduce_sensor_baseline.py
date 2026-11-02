#!/usr/bin/env python3
"""Compile the historical and current filters against the same dropout inputs."""
import argparse
import os
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
BASELINE = "fc1981c16411bc86549692bf2fd8406d9b804a4a"

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",default=shutil.which("g++") or "C:/msys64/ucrt64/bin/g++.exe")
    args=parser.parse_args()
    os.environ["PATH"]=str(pathlib.Path(args.compiler).resolve().parent)+os.pathsep+os.environ.get("PATH","")
    output=ROOT / "build/sensor-analysis"
    output.mkdir(parents=True,exist_ok=True)
    header=subprocess.check_output(["git","show",BASELINE+":./Custom/Inc/HzSensorKalmanFilter.hpp"],cwd=ROOT)
    (output / "legacy_filter.hpp").write_bytes(header)
    for name in ("legacy","fixed"):
        exe=output / (name+"_repro.exe")
        command=[args.compiler,"-std=c++14","-I"+str(output),"-I"+str(ROOT/"Custom/Inc"),
                 str(ROOT/"Custom/Tests/sensor_legacy_repro.cpp"),"-o",str(exe)]
        if name=="legacy": command.append("-DSENSOR_LEGACY")
        subprocess.run(command,cwd=ROOT,check=True)
        result=subprocess.check_output([str(exe)],cwd=ROOT,text=True)
        (output/(name+".txt")).write_text(result,encoding="utf-8")
        (output/(name+".csv")).write_text("\n".join(line for line in result.splitlines() if not line.startswith("#"))+"\n",encoding="utf-8")
        print(name+":\n"+result)
    return 0

if __name__=="__main__":
    raise SystemExit(main())
