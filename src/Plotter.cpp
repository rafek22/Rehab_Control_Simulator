#include "Plotter.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>


void Plotter::addSample(
    double time,
    double referenceAngle,
    double angle,
    double velocity,
    double motorCommand
)
{
    // Conserva todos los valores necesarios para dibujar las graficas al final.
    samples.push_back(
        {
            time,
            referenceAngle,
            angle,
            velocity,
            motorCommand
        }
    );
}


void Plotter::saveAll(
    const std::string& directory
) const
{
    // Sin muestras no existe ninguna curva que representar.
    if (samples.empty())
        return;

    // Crea el directorio de salida y despues genera cada archivo SVG.
    std::filesystem::create_directories(directory);

    PositionPlot(directory + "/position_tracking.svg");

    VelocityPlot(directory + "/velocity.svg");

    MotorPlot(directory + "/motor_command.svg");
}


void Plotter::PositionPlot(const std::string& filename) const
{
    std::ofstream file(filename);

    if (!file.is_open())
        return;

    const double width = 1000.0;
    const double height = 500.0;

    const double left = 80.0;
    const double right = 30.0;
    const double top = 40.0;
    const double bottom = 60.0;

    double minTime = samples.front().time;
    double maxTime = samples.back().time;

    double rawMin =
        std::numeric_limits<double>::max();

    double rawMax =
        std::numeric_limits<double>::lowest();

    for (const Plot& sample : samples)
    {
        rawMin = std::min(rawMin,std::min(sample.referenceAngle,sample.angle));
        rawMax = std::max(rawMax,std::max(sample.referenceAngle,sample.angle));
    }

    const double yStep = 20.0;

    double minValue = std::floor(rawMin / yStep) * yStep;

    double maxValue = std::ceil(rawMax / yStep) * yStep;

    auto mapX = [&](double value)
    {
        return left + (value - minTime) / (maxTime - minTime) * (width - left - right);
    };

    auto mapY = [&](double value)
    {
        return top + (maxValue - value) / (maxValue - minValue) * (height - top - bottom);
    };

    file
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" "
        << "height=\"" << height << "\" "
        << "viewBox=\"0 0 "
        << width << " "
        << height << "\">\n";

    file
        << "<rect width=\"100%\" height=\"100%\" "
        << "fill=\"white\" />\n";


    // =========================
    // Cuadricula y marcas del eje Y
    // =========================

    for (double value = minValue; value <= maxValue + 0.001; value += yStep)
    {
        double y = mapY(value);

        file
            << "<line x1=\"" << left
            << "\" y1=\"" << y
            << "\" x2=\"" << width - right
            << "\" y2=\"" << y
            << "\" stroke=\"#dddddd\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << left - 10
            << "\" y=\"" << y + 5
            << "\" text-anchor=\"end\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << static_cast<int>(value)
            << "</text>\n";
    }


    // =========================
    // Marcas del eje X cada 5 segundos
    // =========================

    const double timeStep = 5.0;

    for (double value = 0.0; value <= maxTime + 0.001; value += timeStep)
    {
        double x = mapX(value);

        file
            << "<line x1=\"" << x
            << "\" y1=\"" << top
            << "\" x2=\"" << x
            << "\" y2=\"" << height - bottom
            << "\" stroke=\"#eeeeee\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << x
            << "\" y=\"" << height - bottom + 20
            << "\" text-anchor=\"middle\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << static_cast<int>(value)
            << "</text>\n";
    }


    // =========================
    // Ejes
    // =========================

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << top
        << "\" x2=\"" << left
        << "\" y2=\"" << height - bottom
        << "\" stroke=\"black\" "
        << "stroke-width=\"1.2\" />\n";

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << height - bottom
        << "\" x2=\"" << width - right
        << "\" y2=\"" << height - bottom
        << "\" stroke=\"black\" "
        << "stroke-width=\"1.2\" />\n";


    // =========================
    // Titulo y etiquetas
    // =========================

    file
        << "<text x=\"" << width / 2
        << "\" y=\"25\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"20\">"
        << "Elbow Position Tracking"
        << "</text>\n";

    file
        << "<text x=\"20\" y=\""
        << height / 2
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\" "
        << "transform=\"rotate(-90 20 "
        << height / 2
        << ")\">Angle [deg]</text>\n";

    file
        << "<text x=\"" << width / 2
        << "\" y=\"" << height - 10
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\">"
        << "Time [s]"
        << "</text>\n";


    // =========================
    // Curva de referencia
    // =========================

    file
        << "<polyline fill=\"none\" "
        << "stroke=\"#d95f02\" "
        << "stroke-width=\"2\" points=\"";

    for (const Plot& sample : samples)
    {
        file
            << mapX(sample.time)
            << ","
            << mapY(sample.referenceAngle)
            << " ";
    }

    file << "\" />\n";


    // =========================
    // Curva del angulo real
    // =========================

    file
        << "<polyline fill=\"none\" "
        << "stroke=\"#1b70c9\" "
        << "stroke-width=\"2\" points=\"";

    for (const Plot& sample : samples)
    {
        file
            << mapX(sample.time)
            << ","
            << mapY(sample.angle)
            << " ";
    }

    file << "\" />\n";


    // =========================
    // Leyenda
    // =========================

    file
        << "<line x1=\"760\" y1=\"60\" "
        << "x2=\"790\" y2=\"60\" "
        << "stroke=\"#d95f02\" "
        << "stroke-width=\"3\" />\n"

        << "<text x=\"800\" y=\"65\" "
        << "font-family=\"Arial\" "
        << "font-size=\"13\">"
        << "Reference"
        << "</text>\n"

        << "<line x1=\"760\" y1=\"82\" "
        << "x2=\"790\" y2=\"82\" "
        << "stroke=\"#1b70c9\" "
        << "stroke-width=\"3\" />\n"

        << "<text x=\"800\" y=\"87\" "
        << "font-family=\"Arial\" "
        << "font-size=\"13\">"
        << "Elbow angle"
        << "</text>\n";

    file << "</svg>\n";
}


void Plotter::VelocityPlot(
    const std::string& filename) const
{
    std::ofstream file(filename);

    if (!file.is_open())
        return;

    const double width = 1000.0;
    const double height = 400.0;

    const double left = 85.0;
    const double right = 30.0;
    const double top = 40.0;
    const double bottom = 60.0;

    double minTime = samples.front().time;
    double maxTime = samples.back().time;

    double maxAbsVelocity = 0.0;

    for (const Plot& sample : samples)
        maxAbsVelocity =std::max(maxAbsVelocity,std::abs(sample.velocity));

    const double yStep = 0.5;

    double maxValue =std::ceil( maxAbsVelocity / yStep) * yStep;

    double minValue = -maxValue;

    auto mapX = [&](double value)
    {
        return left + (value - minTime) / (maxTime - minTime) * (width - left - right);
    };

    auto mapY = [&](double value)
    {
        return top + (maxValue - value) / (maxValue - minValue) * (height - top - bottom);
    };

    file
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" "
        << "height=\"" << height << "\" "
        << "viewBox=\"0 0 "
        << width << " "
        << height << "\">\n";

    file
        << "<rect width=\"100%\" height=\"100%\" "
        << "fill=\"white\" />\n";


    // =========================
    // Cuadricula y marcas del eje Y
    // =========================

    for (double value = minValue; value <= maxValue + 0.001; value += yStep)
    {
        double y = mapY(value);

        file
            << "<line x1=\"" << left
            << "\" y1=\"" << y
            << "\" x2=\"" << width - right
            << "\" y2=\"" << y
            << "\" stroke=\"#dddddd\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << left - 10
            << "\" y=\"" << y + 5
            << "\" text-anchor=\"end\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << value
            << "</text>\n";
    }


    // =========================
    // Cuadricula y marcas del eje X
    // =========================

    const double timeStep = 5.0;

    for (double value = 0.0; value <= maxTime + 0.001; value += timeStep)
    {
        double x = mapX(value);

        file
            << "<line x1=\"" << x
            << "\" y1=\"" << top
            << "\" x2=\"" << x
            << "\" y2=\"" << height - bottom
            << "\" stroke=\"#eeeeee\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << x
            << "\" y=\"" << height - bottom + 20
            << "\" text-anchor=\"middle\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << static_cast<int>(value)
            << "</text>\n";
    }


    // =========================
    // Ejes
    // =========================

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << top
        << "\" x2=\"" << left
        << "\" y2=\"" << height - bottom
        << "\" stroke=\"black\" "
        << "stroke-width=\"1.2\" />\n";

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << mapY(0.0)
        << "\" x2=\"" << width - right
        << "\" y2=\"" << mapY(0.0)
        << "\" stroke=\"#777777\" "
        << "stroke-width=\"1.2\" />\n";


    // =========================
    // Titulo y etiquetas
    // =========================

    file
        << "<text x=\"" << width / 2
        << "\" y=\"25\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"20\">"
        << "Elbow Angular Velocity"
        << "</text>\n";

    file
        << "<text x=\"20\" y=\""
        << height / 2
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\" "
        << "transform=\"rotate(-90 20 "
        << height / 2
        << ")\">Velocity [rad/s]</text>\n";

    file
        << "<text x=\"" << width / 2
        << "\" y=\"" << height - 10
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\">"
        << "Time [s]"
        << "</text>\n";


    // =========================
    // Curva de velocidad
    // =========================

    file
        << "<polyline fill=\"none\" "
        << "stroke=\"#1b70c9\" "
        << "stroke-width=\"2\" points=\"";

    for (const Plot& sample : samples)
    {
        file
            << mapX(sample.time)
            << ","
            << mapY(sample.velocity)
            << " ";
    }

    file << "\" />\n";

    file << "</svg>\n";
}


void Plotter::MotorPlot(
    const std::string& filename) const
{
    std::ofstream file(filename);

    if (!file.is_open())
        return;

    const double width = 1000.0;
    const double height = 400.0;

    const double left = 85.0;
    const double right = 30.0;
    const double top = 40.0;
    const double bottom = 60.0;

    double minTime = samples.front().time;
    double maxTime = samples.back().time;

    const double minValue = -1.0;
    const double maxValue = 1.0;

    const double yStep = 0.2;

    auto mapX = [&](double value)
    {
        return left + (value - minTime) / (maxTime - minTime) * (width - left - right);
    };

    auto mapY = [&](double value)
    {
        return top + (maxValue - value) / (maxValue - minValue) * (height - top - bottom);
    };

    file
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" "
        << "height=\"" << height << "\" "
        << "viewBox=\"0 0 "
        << width << " "
        << height << "\">\n";

    file
        << "<rect width=\"100%\" height=\"100%\" "
        << "fill=\"white\" />\n";


    // =========================
    // Cuadricula y marcas del eje Y
    // =========================

    for (double value = minValue; value <= maxValue + 0.001; value += yStep)
    {
        double cleanValue = std::abs(value) < 0.0001 ? 0.0 : value;

        double y = mapY(cleanValue);

        file
            << "<line x1=\"" << left
            << "\" y1=\"" << y
            << "\" x2=\"" << width - right
            << "\" y2=\"" << y
            << "\" stroke=\"#dddddd\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << left - 10
            << "\" y=\"" << y + 5
            << "\" text-anchor=\"end\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << cleanValue
            << "</text>\n";
    }


    // =========================
    // Cuadricula y marcas del eje X
    // =========================

    const double timeStep = 5.0;

    for (double value = 0.0; value <= maxTime + 0.001; value += timeStep)
    {
        double x = mapX(value);

        file
            << "<line x1=\"" << x
            << "\" y1=\"" << top
            << "\" x2=\"" << x
            << "\" y2=\"" << height - bottom
            << "\" stroke=\"#eeeeee\" "
            << "stroke-width=\"1\" />\n";

        file
            << "<text x=\"" << x
            << "\" y=\"" << height - bottom + 20
            << "\" text-anchor=\"middle\" "
            << "font-family=\"Arial\" "
            << "font-size=\"12\">"
            << static_cast<int>(value)
            << "</text>\n";
    }


    // =========================
    // Ejes
    // =========================

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << top
        << "\" x2=\"" << left
        << "\" y2=\"" << height - bottom
        << "\" stroke=\"black\" "
        << "stroke-width=\"1.2\" />\n";

    file
        << "<line x1=\"" << left
        << "\" y1=\"" << mapY(0.0)
        << "\" x2=\"" << width - right
        << "\" y2=\"" << mapY(0.0)
        << "\" stroke=\"#777777\" "
        << "stroke-width=\"1.2\" />\n";


    // =========================
    // Titulo y etiquetas
    // =========================

    file
        << "<text x=\"" << width / 2
        << "\" y=\"25\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"20\">"
        << "Motor Control Command"
        << "</text>\n";

    file
        << "<text x=\"20\" y=\""
        << height / 2
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\" "
        << "transform=\"rotate(-90 20 "
        << height / 2
        << ")\">Command [-1, 1]</text>\n";

    file
        << "<text x=\"" << width / 2
        << "\" y=\"" << height - 10
        << "\" "
        << "text-anchor=\"middle\" "
        << "font-family=\"Arial\" "
        << "font-size=\"14\">"
        << "Time [s]"
        << "</text>\n";


    // =========================
    // Curva de la orden del motor
    // =========================

    file
        << "<polyline fill=\"none\" "
        << "stroke=\"#1b70c9\" "
        << "stroke-width=\"2\" points=\"";

    for (const Plot& sample : samples)
    {
        file
            << mapX(sample.time)
            << ","
            << mapY(sample.motorCommand)
            << " ";
    }

    file << "\" />\n";

    file << "</svg>\n";
}
