# Hexumi
---

Hexumi is a low-cost 3D-printed 18 DoF walking hexapod robot built using Raspberry Pi and ROS2.

<img src="/img/HEXUMI_DSSERVO-1.png" height="300" />

## Acknowledgements
The code for Pimoroni Servo2040 servo driver is based on the [C/C++ Pimoroni Pico library](https://github.com/pimoroni/pimoroni-pico) and [micro-ROS](https://github.com/micro-ROS/).

## Documentation

I am currently working on the documentation, assembly instructions, and installation guide, which will be available shortly.

## Development
---

For in-depth insights into the development process and detailed build instructions, please refer to my 2023 paper, "Design Process of an 18 DoF Walking Hexapod Robot," accessible on my [ResearchGate profile](https://www.researchgate.net/profile/Maksymilian-Firkowski/research). Even if you don't intend to replicate Hexumi, the paper provides a valuable roadmap for designing your robot.

## Why Hexumi?
I designed Hexumi as the subject of my Mechanical Engineering BSc thesis at the Lodz University of Technology, driven by two significant events: the COVID-19 pandemic and the rise of consumer AI. Over the past years, millions of people lost the ability to interact with the outside world due to lockdowns. Simultaneously, the threat of machine learning systems replacing online work loomed. Existing robotic tools for physical labor were either too expensive or lacked sophistication, making them inaccessible to the average person. Thus, I aimed to contribute to the development of affordable robotic solutions for research, education, and entertainment, with the potential to democratize access to mobile and dextrous robots.

### Key Features

- **Cost-efficiency and Accessibility:** Hexumi is designed for easy replication using consumer-grade 3D printing techniques and off-the-shelf electronic components. Depending on your circumstances and 3D printer availability, the build cost can be as low as 300 EUR.

- **Material-efficiency and Sustainability:** The mechanical elements require minimal 3D printing supports, reducing material waste and improving sustainability. Its modular structure ensures that you only need to print minor components when replacements are necessary.

- **Modifiability and Repairability:** Hexumi's components can be easily replaced, whether due to failure or modification. In many cases, only a small amount of material needs to be used for replacements. For example, if you want to change the ground-contact sensors, you only need to modify the tips of the tibias, not the entire links.

- **Software and Sensor Expandability:** Hexumi is designed to operate with the Robot Operating System, preferably ROS2, as its RP2040-based servo driver is well-suited for Micro-ROS. The robot's housing design, a 5V USB power port, and 1.5kg static payload capacity allow mounting additional sensors, such as LIDARs, cameras, IMUs, and more.

- **Ease of Use:** Hexumi was designed to be both modifiable and easy to use. Peripheral components, such as the charging port, provide a simple out-of-the-box experience for a novice user.

## Licensing
---

CAD files, STL files, and the software are shared under the Apache 2.0 license. For details, see [LICENSE](LICENSE).

## About The Author
---

My name is Maksymilian Firkowski (<i>feer-cov-skee, 피르코브스키</i>). I am a young Mechanical Engineer born in Poland interested in the development of intelligent mechanical systems that are reliable and sustainable at the mechanical and software levels. I happen to be temperamentally inclined towards low-level, bare-metal design where I can understand the inner workings of a system; that's why I've wanted to be a mechanical engineer since childhood and perhaps that's also the reason why I lean towards low-level software development. I am an avid believer in the open-source philosophy and technological education, and do my best to support them. In my free time I work on projects similar to this one, with Hexumi being my most extensive work yet. I also dabble in industrial, design, graphical design, and cryptography.
