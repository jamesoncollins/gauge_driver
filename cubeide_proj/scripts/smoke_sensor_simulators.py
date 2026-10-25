#!/usr/bin/env python3
"""Smoke-test native commands and browser sensor exports, CSV, and canvas."""
import csv
import io
import json
import os
import pathlib
import subprocess
import tempfile
import time

import capture_web_frames as capture

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT / "build/sensor-analysis"

class RecordingClient(capture.CdpClient):
    def __init__(self,url):
        self.logs=[]; self.errors=[]
        super().__init__(url)
    def recv_msg(self,timeout=10):
        message=super().recv_msg(timeout)
        if message.get("method")=="Runtime.consoleAPICalled":
            params=message["params"]
            line=" ".join(str(arg.get("value","")) for arg in params.get("args",[]))
            self.logs.append(line)
            if params.get("type")=="error": self.errors.append(line)
        if message.get("method")=="Runtime.exceptionThrown": self.errors.append(str(message))
        return message

def evaluate(client,expression):
    response=client.command("Runtime.evaluate",{"expression":expression,"returnByValue":True})
    result=response["result"]
    if "exceptionDetails" in result: raise RuntimeError(str(result))
    return result["result"].get("value")

def native():
    startup=subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow=0
    commands=[f"sensor {s} {c}" for s in ("clean","missing","dropout","extra","jitter","stall")
              for c in ("speed","tach","both")]
    commands += ["layout cruise","sensor reset","sensor nonsense","sensor stall both"]
    with (OUT/"native-stdout.txt").open("w") as stdout, (OUT/"native-stderr.txt").open("w") as stderr:
        process=subprocess.Popen([str(ROOT/"build/x86-debug/gauge_driver.exe")],cwd=ROOT,
            stdin=subprocess.PIPE,stdout=stdout,stderr=stderr,text=True,startupinfo=startup)
        try:
            time.sleep(0.5)
            process.stdin.write("\n".join(commands)+"\n"); process.stdin.flush()
            time.sleep(4.5)
            process.stdin.write("sensor trace\n"); process.stdin.flush()
            time.sleep(0.2)
            if process.poll() is not None: raise RuntimeError("native simulator exited unexpectedly")
        finally:
            process.terminate(); process.wait(timeout=5)
    log=(OUT/"native-stderr.txt").read_text()
    for command in commands:
        prefix="sim command not recognized: " if command=="sensor nonsense" else "sim command accepted: "
        assert prefix+command in log,command
    assert "now_us,frame_us,channel" in (OUT/"native-stdout.txt").read_text()
    print("native: scenario, reset, trace, and existing layout controls passed")

def web():
    with tempfile.TemporaryDirectory(prefix="gauge-sensor-smoke-") as directory:
        with capture.serve_directory(ROOT/"build/web-debug",0) as url:
            proc,port=capture.start_chrome(capture.find_chrome(None),pathlib.Path(directory))
            client=None
            try:
                client=RecordingClient(capture.open_page(port,url))
                client.command("Runtime.enable"); client.command("Page.enable")
                capture.wait_for_canvas(client)
                for scenario in range(6):
                    for mask in range(1,4):
                        assert evaluate(client,f"Module._sim_control_sensor_scenario({scenario},{mask})")==1
                assert evaluate(client,"Module._sim_control_sensor_scenario(99,3)")==0
                evaluate(client,"Module._sim_control_sensor_reset()")
                evaluate(client,"Module._sim_control_sensor_scenario(5,3)")
                time.sleep(4.5)
                evaluate(client,"Module._sim_control_sensor_trace(-1)")
                lines=[line for line in client.logs if line.startswith("now_us,") or
                       (line and line[0].isdigit() and "," in line)]
                text="\n".join(lines)+"\n"
                (OUT/"web-stall.csv").write_text(text,encoding="utf-8")
                rows=list(csv.DictReader(io.StringIO(text)))
                assert rows and any(int(row["queue_drops"])>0 for row in rows)
                for row in rows[-2:]: assert int(row["stale"])==0,row
                capture.set_sim_layout(client,1)
                frames=capture.capture_frames(client,ROOT/"build/web-debug/analysis",12,0.75)
                (ROOT/"build/web-debug/analysis/frames.json").write_text(json.dumps(frames,indent=2))
                assert all(frame["width"]==240 and frame["height"]==320 for frame in frames)
                # SDL initialization requests swap timing before the existing
                # host entrypoint installs its main loop. Preserve that unrelated
                # startup diagnostic while rejecting all other browser errors.
                known="emscripten_set_main_loop_timing: Cannot set timing mode for main loop since a main loop does not exist! Call emscripten_set_main_loop first to set one up."
                (OUT/"web-runtime-log.txt").write_text("\n".join(client.logs),encoding="utf-8")
                unexpected=[error for error in client.errors if error!=known]
                assert not unexpected,unexpected
            finally:
                if client: client.close()
                proc.terminate()
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=5)
    print("web: exports, stall recovery, CSV, layout control, and twelve 240x320 frames passed")

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (ROOT/"build/web-debug/analysis").mkdir(parents=True,exist_ok=True)
    os.environ["PATH"]="C:/msys64/ucrt64/bin"+os.pathsep+os.environ.get("PATH","")
    native(); web()
    return 0

if __name__=="__main__": raise SystemExit(main())
