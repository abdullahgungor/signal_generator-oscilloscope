# signal_generator-oscilloscope
 STM32F103C8T6 Based signal generator & oscilloscope project.


TR: STM32F103C8T6 Tabanlı sinyal üreteci & osiloskop projesi. Şu anda 100 hz ve 200 hz çıkış verebilmektedir. Fakat ileriki sürümlerde daha da yüksek frekanslarda çıkış vermesi hedeflenmektedir. Analog sinyal 8 Bit R-2R (Resistor Ladder) ile oluşturulmaktadır. Python ile yazılan arayüz sayesinde analog pin üzerinden okunan veriler grafikselleştirilir. Arayüz üzerinden dalga ve frekans seçimi yapılmaktadır. Arayüz şimdilik sadece Windows'u destekliyor. Duty Cycle ayarlanabilir PWM ve Kare, Sinüs, Üçgen, Testere tipinde dalga çıkışı üretebilmektedir şuan.

EN: STM32F103C8T6 Based signal generator & oscilloscope project. It can currently output 100 hz and 200 hz. But it is aimed to output at even higher frequencies in future versions. Analog signal is created with 8 Bit R-2R (Resistor Ladder). Thanks to the interface written in Python, the data read over the analog pin is graphicalized. Wave and frequency selection is made through the interface. For now, the interface only supports Windows. Duty Cycle adjustable PWM and Square, Sine, Triangle, Saw type wave output can be produced now.