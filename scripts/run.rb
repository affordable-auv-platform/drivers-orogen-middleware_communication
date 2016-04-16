require 'orocos'
require 'readline'

include Orocos

Orocos.initialize

Orocos.run "middleware_communication::Task" => "middleware_comm" do
    task = TaskContext.get 'middleware_comm'
    task.configure
    task.start

    imu_sensors_reader = task.imu_sensors_samples.reader
    temperature_reader = task.temperature_samples.reader
    pressure_bar_reader = task.pressure_bar_samples.reader

    while true
        if imu_sensors = imu_sensors_reader.read_new
            puts "imu sensors: #{imu_sensors.time}"
            puts "acc:  #{imu_sensors.acc}"
            puts "gyro: #{imu_sensors.gyro}"
            puts "mag:  #{imu_sensors.mag}"
        end
        if temperature = temperature_reader.read_new
            puts "temperature: #{temperature.kelvin}"
        end
        if pressure = pressure_bar_reader.read_new
            puts "pressure: #{pressure}"
        end
        sleep 0.5
    end

    Readline.readline("Press ENTER to exit\n")
end
