graph LR
    %% Power Block
    Bat("[🔋 3x 18650 Battery]")
    Sw("[🔌 Switch]")
    Buck("[⚡ 5V Buck Converter]")

    %% Control Block
    ESP("[🧠 ESP32]")
    L298N("[⚙️ L298N Driver]")

    %% Motors
    ML("[(Left BO Motor)]")
    MR("[(Right BO Motor)]")

    %% Power Wiring
    Bat -- "Red (+)" --> Sw
    Bat -- "Black (GND)" --> L298N
    Bat -- "Black (GND)" --> Buck
    Sw -- "Out (+)" --> L298N
    Sw -- "Out (+)" --> Buck

    Buck -- "OUT+ (5V)" --> ESP
    Buck -- "OUT- (GND)" --> ESP

    %% Logic Wiring (ESP32 to L298N)
    ESP -- "GPIO 25" -->|ENA| L298N
    ESP -- "GPIO 26" -->|IN1| L298N
    ESP -- "GPIO 27" -->|IN2| L298N
    ESP -- "GPIO 32" -->|IN3| L298N
    ESP -- "GPIO 14" -->|IN4| L298N
    ESP -- "GPIO 33" -->|ENB| L298N

    %% Motor Wiring
    L298N -- "OUT1, OUT2" --> ML
    L298N -- "OUT3, OUT4" --> MR

    classDef power fill:#f9d0c4,stroke:#333,stroke-width:2px;
    classDef logic fill:#d4e6f1,stroke:#333,stroke-width:2px;
    classDef motor fill:#d5f5e3,stroke:#333,stroke-width:2px;
    
    class Bat,Sw,Buck power;
    class ESP,L298N logic;
    class ML,MR motor;
