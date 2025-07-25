# Voice-Controlled Robotic Hand Bottle Opener

**AI to Robotica — College Project**
*Simulate a robotic hand opening a bottle cap with voice commands in PyBullet*

---

## Overview

This project demonstrates the fusion of **AI (speech recognition)** and **robotics simulation**.
You control a realistic robotic hand in a PyBullet simulation by simply saying voice commands like “open”, “rest”, “big”, “small”, or “reset”. The hand can open a bottle cap using the index finger and thumb only, twist it off, and lift away—just like a human!

---

## Features

* **Voice Command AI:**
  Uses real-time speech recognition to control the hand (via the `SpeechRecognition` Python library and Google Speech API).

* **Realistic Robotic Hand Simulation:**
  Shadow Hand model in PyBullet with smooth finger/joint animation.

* **Bottle & Cap Physics:**
  Simulated bottle and cap as independent bodies. The cap is twisted, lifted, and then falls naturally with gravity.

* **Multiple Bottles:**
  Supports switching hand alignment for different bottle heights (“big”/“small” commands).

* **Modular, Well-Documented Code:**
  All main logic is organized for easy understanding and extension.

---

## Voice Commands

* `open` – Simulate pinching, twisting, and lifting the cap.
* `rest` – Hand returns to relaxed (open palm) pose.
* `big`  – Move hand to align with a taller bottle.
* `small` – Move hand to align with a shorter bottle.
* `reset` – Smoothly reset hand to the base position.

---

## Requirements

* **Python 3.8+**
* [PyBullet](https://pybullet.org/)
* [SpeechRecognition](https://pypi.org/project/SpeechRecognition/)
* [PyAudio](https://pypi.org/project/PyAudio/) (for microphone input)
* [trimesh](https://trimsh.org/) (for loading mesh files)
* [threading](https://docs.python.org/3/library/threading.html) (standard library)

Install dependencies via pip:

pip install -r requirements.txt


> **Note:**
> On Linux, you may need `sudo apt install portaudio19-dev` before installing PyAudio.

---

## File Structure

* `main.py`
  Main entry point: sets up environment, loads hand/bottle, runs voice command loop.

* `resources/`
  Contains all 3D models (URDFs, meshes for hand, bottle, and cap).

* *(If split)*

  * `robotic_hand.py` — All pose, motion, and animation logic for the hand/cap, joint IDs, file paths, etc.
  * `voice_utils.py` — Voice recognition thread and command dispatcher.
---

## How to Run

1. **Prepare Resources:**
   Make sure you have the required hand and bottle URDF/mesh files in `resources/`.

2. **Start the Simulation:**

   ```bash
   python main.py
   ```

   (Or the entrypoint you named.)

3. **Speak a Command:**
   When prompted (“Say ‘open’ or ‘rest’...”), say any supported command.
   The hand and cap will animate accordingly!

---

## Example Usage

* Say “open” to see the hand pinch, twist, and lift the cap.
* Say “rest” to relax the hand.
* Say “big” or “small” to move the hand for a big/small bottle size.
* Say “reset” to return hand to the base position.

---

## AI/ML Component

This project uses **AI-powered speech recognition** (Google Speech API via the `SpeechRecognition` library) to interpret natural language commands in real time, creating an interactive and intuitive control system.

---

## Troubleshooting

* If you get audio or `PyAudio` errors, ensure your microphone is connected and the correct drivers (and PortAudio) are installed.
* On Linux, check `arecord` and `aplay` work.
* You can list available microphones in Python with:

  ```python
  import speech_recognition as sr
  print(sr.Microphone.list_microphone_names())
  ```
* If simulation is slow, reduce the number of animation steps or TIME\_STEP value.

---

## References

* [PyBullet](https://pybullet.org/)
* [SpeechRecognition](https://pypi.org/project/SpeechRecognition/)
* [Shadow Hand ign Model](https://github.com/AndrejOrsula/shadow_hand_ign)
* [trimesh](https://trimsh.org/)
* [plastic bottle urdf](https://sketchfab.com/3d-models/plastic-water-bottle-f94e72b7c06446cc85816492af416f8f)

---

## License

This code is for educational purposes.
*If you use Shadow Hand or bottle assets, please credit the original authors and abide by their licenses.*


