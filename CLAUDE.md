# CLAUDE.md

## Project

Falcon Gaze — стартовий репозиторій для хакатону з оптичною комунікацією дрон-роя в Gazebo/PX4/ROS 2.
Завдання: дрон-лідер (drone 0) знає маршрут; фолловер-дрони (1, 2, 3) не мають GPS і слідують за лідером
тільки через камеру та LED-сигнали.

## Important Docs

Прочитай перед будь-якими змінами поведінки:

- `docs/README.md`
- `docs/overview.md`
- `docs/getting-started.md`
- `docs/hackathon-rules.md`
- `docs/sdk-api.md`
- `docs/examples.md`
- `docs/evaluation.md`
- `SDK_AND_EXAMPLES.md`
- `README_setup.md`
- `Хакатон_ГЕНЕРА_2_документація-1.pdf` — офіційне джерело правил хакатону

## Repository Structure

- `drone_sdk/` — Python SDK поверх MAVSDK + ROS/Gazebo bridges для камери та LED.
- `examples/` — приклади для одного дрона та рою з 4 дронів.
- `resources/scripts/` — launch-скрипти PX4/Gazebo та місійні скрипти.
- `resources/plugins/led_controller/` — Gazebo system plugin для LED-команд.
- `resources/worlds/` — кастомний світ baylands_custom та медіа.
- `resources/x500_base/`, `resources/x500_mono_cam/` — оверрайди моделей для PX4.
- `project_setup.sh` — копіює ресурси репо в `~/PX4-Autopilot/Tools/simulation/gz`.

## Environment

Базова конфігурація: Windows 11 + WSL2 + Ubuntu 22.04, Gazebo Harmonic 8.x,
PX4 `v1.15.4`, ROS 2 Humble.

**Реальний шлях репо в WSL:**
```
/mnt/c/Users/masar/genera/Henera_hackaton_kleparivska_spilka
```
(У AGENTS.md написано `~/falcon_gaze` — це застаріло, використовуй шлях вище.)

Кожен термінал потребує:
```bash
source /opt/ros/humble/setup.bash
```

Перед запуском симуляції:
```bash
source /mnt/c/Users/masar/genera/Henera_hackaton_kleparivska_spilka/resources/scripts/px4_gz_setup.sh
```

## Key Topics

| Призначення | Topic |
|---|---|
| Камера дрона N | `/world/baylands_custom/model/x500_mono_cam_N/link/mono_cam/base_link/sensor/camera_sensor/image` |
| LED команда дрону N | `/model/x500_mono_cam_N/led_cmd` |

`N` = 0 (лідер), 1, 2, 3 (фолловери).

LED маски: `1100` (`FOLLOW`), `1000` (`HOLD`), `0100` (`SAFE`), `0000` (`FINISH` off-фаза), `ON`, `OFF`, `BLINK`.

**LED-бюджет:** модель має **2 лінзи** (`led_lens_01`, `led_lens_04`) — одна LED-група на лідері
(вимога правил). Default-матеріал темний; лінзи світяться лише коли лідер шле команду.
Фолловери LED-команд не шлють → їхні лінзи лишаються темні (фактично без LED).
Активні лише перші 2 біти маски (3-й і 4-й ігноруються — лінз тільки 2).

**Кольори (для CV):** `led_lens_01` (index 0) = **зелений** LED,
`led_lens_04` (index 1) = **червоний** LED. Колір задає плагін
`LedController.cc` за індексом; runtime-команди лише вмикають або вимикають лінзи.
Протокол: `FOLLOW` = зелений+червоний, `HOLD` = зелений, `SAFE` = червоний,
`FINISH` = зелений+червоний блимають разом в одній фазі.
навіть коли вони накладаються (без злиття в одну пляму). Після зміни плагіна — перезібрати.

## MAVSDK Ports

| Дрон | UDP | gRPC |
|---|---|---|
| 0 (лідер) | 14540 | 50051 |
| 1 | 14541 | 50052 |
| 2 | 14542 | 50053 |
| 3 | 14543 | 50054 |

## Common Commands

**Підготовка (один раз після `make px4_sitl gz_x500`):**
```bash
# Виправити CRLF перед запуском (файл із Windows)
sed -i 's/\r//' project_setup.sh
bash project_setup.sh

# Зібрати LED плагін
cd ~/PX4-Autopilot/Tools/simulation/gz/plugins/led_controller
rm -rf build && mkdir build && cd build && cmake .. && make
```

**Запуск симуляції (Термінал 1):**
```bash
export DISPLAY=:0
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
REPO="/mnt/c/Users/masar/genera/Henera_hackaton_kleparivska_spilka"
source "$REPO/resources/scripts/px4_gz_setup.sh"
source /opt/ros/humble/setup.bash
ros2 launch "$REPO/resources/scripts/swarn_launch.py"
```

**Запуск SDK (Термінал 2):**
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/masar/genera/Henera_hackaton_kleparivska_spilka
python3 examples/demo.py          # один дрон
python3 examples/swarm_waypoints.py  # 4 дрони
```

**Місія лідера:**
```bash
cd /mnt/c/Users/masar/genera/Henera_hackaton_kleparivska_spilka/resources/scripts
python3 mission_launch.py
```

## Development Rules

- Зберігай простий SDK-інтерфейс, якщо задача не вимагає ширших змін.
- Фолловери повинні використовувати тільки камеру та оптичний LED-сигнал — не GPS, не ground-truth позицію лідера.
- Не додавай шорткати, що читають глобальну позицію лідера для навігації фолловера (хіба що явно позначено як debug/eval).
- Не комітити `__pycache__/`, `.ulg` логи та інші рантайм-артефакти.
- Зберігай типографіку існуючих файлів (`swarn_launch.py` — навмисна помилка, не перейменовуй).

## Hackathon Constraints (критично)

Фолловери **можуть** читати:
- власні camera frames
- декодовані LED/оптичні повідомлення з камери
- власні IMU/telemetry (attitude, altitude, velocity, local state) через SDK

Фолловери **не можуть** читати:
- global GPS / world coordinates
- позицію/позу лідера напряму
- ROS/Gazebo/PX4 topics що розкривають стан лідера
- будь-який прямий цифровий канал від лідера до фолловера поза оптичним каналом

## Known Gaps

- `evaluate.py` згадується в PDF, але не існує в репо.
- `swarn_launch.py` — typo у назві файлу, зберігай як є.
- Стартовий пакет дає інфраструктуру, але не готове CV/control рішення.
