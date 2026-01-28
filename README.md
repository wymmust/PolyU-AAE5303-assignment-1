# AAE5303 Environment Setup Report — Template for Students

> **Important:** Follow this structure exactly in your submission README.  
> Your goal is to demonstrate **evidence, process, problem-solving, and reflection** — not only screenshots.

---

## 1. System Information

**Laptop model:**  
ROG Zephyrus G14

**CPU / RAM:**  
AMD Ryzen 9 5900HS, 16GB RAM

**Host OS:**  
Windows 11

**Linux/ROS environment type:**  
_[Choose one:]_
- [ ] Dual-boot Ubuntu
- [✓] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment: I created an isolated Python virtual environment using Python’s built-in `venv` module to avoid conflicts with global packages

**Tool used:**  
- **Operating System:** Windows 11 with WSL2
- **Linux Distribution:** Ubuntu 22.04 LTS
- **ROS Distribution:** ROS 2 Humble Hawksbill
- **Python:** Python 3.10.12
- **ROS Build Tool:** colcon
- **Package Management:** apt, pip
- **Python Libraries:**
  - NumPy
  - SciPy
  - Matplotlib
  - OpenCV
  - Open3D
- **Version Control:** Git and GitHub
- **Terminal Environment:** Ubuntu terminal (WSL2)
- **Text Editor:** Visual Studio Code
- **Generative AI Tool:** ChatGPT (used for troubleshooting and guidance)

**Key commands you ran:**
```python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
Minor deviations were required due to network and proxy restrictions in the WSL environment.  
Specifically, `rosdep update` could not be completed because GitHub raw URLs were inaccessible.  
Instead, required ROS 2 dependencies were installed manually using `apt`, and the workspace was successfully built using `colcon build`.  
All functional checks and runtime verification were completed as required._

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
========================================
AAE5303 Environment Check (Python + ROS)
Goal: help you verify your environment and understand what each check means.
========================================

Step 1: Environment snapshot
  Why: We capture platform/Python/ROS variables to diagnose common setup mistakes (especially mixed ROS env).
Step 2: Python version
  Why: The course assumes Python 3.10+; older versions often break package wheels.
Step 3: Python imports (required/optional)
  Why: Imports verify packages are installed and compatible with your Python version.
Step 4: NumPy sanity checks
  Why: We run a small linear algebra operation so success means more than just `import numpy`.
Step 5: SciPy sanity checks
  Why: We run a small FFT to confirm SciPy is functional (not just installed).
Step 6: Matplotlib backend check
  Why: We generate a tiny plot image (headless) to confirm plotting works on your system.
Step 7: OpenCV PNG decoding (subprocess)
  Why: PNG decoding uses native code; we isolate it so corruption/codec issues cannot crash the whole report.
Step 8: Open3D basic geometry + I/O (subprocess)
  Why: Open3D is a native extension; ABI mismatches can segfault. Subprocess isolation turns crashes into readable failures.
Step 9: ROS toolchain checks
  Why: The course requires ROS tooling. This check passes if ROS 2 OR ROS 1 is available (either one is acceptable).
  Action: building ROS 2 workspace package `env_check_pkg` (this may take 1-3 minutes on first run)...
  Action: running ROS 2 talker/listener for a few seconds to verify messages flow...
Step 10: Basic CLI availability
  Why: We confirm core commands exist on PATH so students can run the same commands as in the labs.

=== Summary ===
✅ Environment: {
  "platform": "Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.35",
  "python": "3.10.12",
  "executable": "/home/wym/aae5303-env-check/.venv/bin/python",
  "cwd": "/home/wym/aae5303-env-check",
  "ros": {
    "ROS_VERSION": "2",
    "ROS_DISTRO": "humble",
    "ROS_ROOT": null,
    "ROS_PACKAGE_PATH": null,
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "CMAKE_PREFIX_PATH": null
  }
}
✅ Python version OK: 3.10.12
✅ Module 'numpy' found (v2.2.6).
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.13.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.13.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 detected: rosversion -d -> humble
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /home/wym/aae5303-env-check/.venv/bin/python3

All checks passed. You are ready for AAE5303 🚀
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
ℹ️ Loading /home/wym/aae5303-env-check/data/sample_pointcloud.pcd ...
✅ Loaded 8 points.
   • Centroid: [0.025 0.025 0.025]
   • Axis-aligned bounds: min=[0. 0. 0.], max=[0.05 0.05 0.05]
✅ Filtered point cloud kept 7 points.
✅ Wrote filtered copy with 7 points to /home/wym/aae5303-env-check/data/sample_pointcloud_copy.pcd
   • AABB extents: [0.05 0.05 0.05]
   • OBB  extents: [0.08164966 0.07071068 0.05773503], max dim 0.0816 m
🎉 Open3D point cloud pipeline looks good.
```

**Screenshot:**  
<img width="1355" height="1262" alt="967ccdf10579f6a2233ad9a6b23b8e6f" src="https://github.com/user-attachments/assets/62703d6e-538a-49ec-965e-3cfa53c8a32a" />


---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
colcon build
Starting >>> env_check_pkg
Finished <<< env_check_pkg [12.2s]                       

Summary: 1 package finished [12.4s]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[talker-1] [INFO] [1769417755.196558071] [env_check_pkg_talker]: AAE5303 talker ready (publishing at 2 Hz).
[talker-1] [INFO] [1769417755.696716602] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #0'
[talker-1] [INFO] [1769417756.179878814] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #1'
[talker-1] [INFO] [1769417756.663497394] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #2'
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[listener-2] [INFO] [1769419207.045363463] [env_check_pkg_listener]: AAE5303 listener awaiting messages.
[talker-1] [INFO] [1769419207.099987458] [env_check_pkg_talker]: AAE5303 talker ready (publishing at 2 Hz).
[listener-2] [INFO] [1769419207.542062793] [env_check_pkg_listener]: I heard: 'AAE5303 hello #19'
[talker-1] [INFO] [1769419207.600154566] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #0'
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

<img width="1411" height="1138" alt="image" src="https://github.com/user-attachments/assets/74140488-02cd-4d0f-9a58-b2e67f690beb" />


---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: Colcon build failed with ModuleNotFoundError: No module named 'catkin_pkg

**Cause / diagnosis:**  
The error occurred because colcon build was executed while a Python virtual environment (.venv) was active.
As a result, ROS 2 build scripts were executed using the virtual environment’s Python interpreter, which did not include the catkin_pkg dependency required by ament_cmake.

**Fix:**  
I deactivated the Python virtual environment before building the ROS 2 workspace and then rebuilt the workspace using the system Python environment.

```bash
deactivate
cd ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
```

**Reference:**  
Official ROS 2 documentation and ChatGPT assistance.

---

### Issue 2: Ros2 run env_check_pkg listener.py returned No executable found

**Cause / diagnosis:**  
The package env_check_pkg does not register individual executables for ros2 run.
Instead, both the talker and listener nodes are designed to be launched together using a ROS 2 launch file.

**Fix:**  
I used the provided launch file to start both nodes simultaneously, which successfully verified message communication between the talker and listener.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch env_check_pkg env_check.launch.py
```

**Reference:**  
env_check_pkg repository documentation and ROS 2 launch system documentation.

---

### Issue 3 (Optional): [Title]

**Cause / diagnosis:**  
_[Explain what you think caused it]_

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
[Your fix command/code here]
```

**Reference:**  
_[Official ROS docs? StackOverflow? AI assistant? Something else?]_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
When running git clone inside WSL, the command keeps hanging at "Cloning into ..." and eventually fails with:
fatal: unable to access 'https://github.com/...': Failed to connect to 172.24.208.1:7897 (Connection timed out).
However, ping github.com works normally. Why does this happen and how can I fix it?

```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
This issue is caused by proxy configuration. Although basic network connectivity works, Git’s HTTPS traffic is being routed through a proxy (172.24.208.1:7897) that is not reachable from WSL.
You can diagnose this by checking environment variables such as http_proxy, https_proxy, and ALL_PROXY.
Temporarily unsetting these proxy variables in the terminal and then re-running git clone should resolve the problem.

```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
I followed the AI’s diagnosis that the failure was caused by inherited proxy environment variables inside WSL.
Before applying the fix, I verified the hypothesis by running env | grep -i proxy and confirmed that http_proxy/https_proxy/ALL_PROXY were set to an unreachable proxy address.
I did not change any system-wide proxy settings permanently; instead, I used unset in the current terminal session to avoid unexpected side effects on other tools.
After unsetting the variables, I re-ran git clone and confirmed it completed successfully.

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
env | grep -i proxy
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
git clone --depth 1 https://github.com/qmohsu/PolyU-AAE5303-env-smork-test.git aae5303-env-check

```

**Why this worked:**  
Because Git was previously forced to route HTTPS traffic through a proxy (172.24.208.1:7897) that was not reachable from WSL, causing a timeout.
Unsetting the proxy environment variables allowed Git to connect directly to GitHub and complete the clone operation.

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

This assignment helped me understand how different components of a robotics development environment—Python, ROS 2, system dependencies, and networking—interact with each other.
I was surprised by how non-obvious issues such as proxy settings and Python virtual environments could silently affect ROS 2 build and runtime behavior.
Through debugging these problems, I learned to rely more on reading error messages carefully and validating assumptions step by step instead of repeatedly retrying commands.
Next time, I would set up the environment more systematically and document configuration changes earlier to avoid repeated troubleshooting.
Overall, I now feel significantly more confident in diagnosing and resolving ROS 2 and Python environment issues independently.

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
WU Yumeng

**Student ID:**  
25041983g

**Date:**  
2026/1/28

---

## Submission Checklist

Before submitting, ensure you have:

- [✓] Filled in all system information
- [✓] Included actual terminal outputs (not just screenshots)
- [✓] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [✓] Documented 2–3 real problems with solutions
- [✓] Completed the AI usage section with exact prompts
- [✓] Written a thoughtful reflection (3–5 sentences)
- [✓] Signed the declaration

---

**End of Report**
