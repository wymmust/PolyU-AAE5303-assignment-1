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
[Paste your actual terminal output here]
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]_

![Python Tests Passing](path/to/your/screenshot.png)

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
[Paste your build summary here]
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
[Paste 3-4 lines of talker output here]
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[Paste 3-4 lines of listener output here]
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

![Talker and Listener Running](path/to/your/screenshot.png)

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: [Write the exact error message or problem]

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

### Issue 2: [Another real error or roadblock]

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
[Copy-paste your actual message to the AI, not a summary]
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
[Quote only the relevant part of the AI's answer]
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
_[Write your analysis here]_

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
[Your final command/code here]
```

**Why this worked:**  
_[Brief explanation]_

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

_[Write your 3-5 sentence reflection here]_

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[Your name]_

**Student ID:**  
_[Your student ID]_

**Date:**  
_[Date of submission]_

---

## Submission Checklist

Before submitting, ensure you have:

- [ ] Filled in all system information
- [ ] Included actual terminal outputs (not just screenshots)
- [ ] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [ ] Documented 2–3 real problems with solutions
- [ ] Completed the AI usage section with exact prompts
- [ ] Written a thoughtful reflection (3–5 sentences)
- [ ] Signed the declaration

---

**End of Report**
