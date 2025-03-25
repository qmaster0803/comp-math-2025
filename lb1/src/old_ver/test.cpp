#include <boost/numeric/odeint.hpp>
#include <iostream>

using namespace boost::numeric::odeint;

// Класс, представляющий состояние системы
class HarmonicOscillator {
public:
    // Конструктор
    HarmonicOscillator(double x0 = 0.0, double v0 = 0.0) : x(x0), v(v0) {}

    // Оператор доступа к элементам
    double& operator[](size_t i) {
        if (i == 0) return x;
        if (i == 1) return v;
        throw std::out_of_range("HarmonicOscillator has only 2 elements");
    }

    // Константный оператор доступа к элементам
    const double& operator[](size_t i) const {
        if (i == 0) return x;
        if (i == 1) return v;
        throw std::out_of_range("HarmonicOscillator has only 2 elements");
    }

    // Метод для получения размера
    size_t size() const {
        return 2;
    }

    // Арифметические операции
    HarmonicOscillator& operator+=(const HarmonicOscillator& other) {
        x += other.x;
        v += other.v;
        return *this;
    }

    HarmonicOscillator& operator*=(double scalar) {
        x *= scalar;
        v *= scalar;
        return *this;
    }

    // Дружественные функции для арифметических операций
    friend HarmonicOscillator operator+(HarmonicOscillator lhs, const HarmonicOscillator& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend HarmonicOscillator operator*(HarmonicOscillator lhs, double scalar) {
        lhs *= scalar;
        return lhs;
    }

    friend HarmonicOscillator operator*(double scalar, HarmonicOscillator rhs) {
        rhs *= scalar;
        return rhs;
    }

    // Оператор вызова функции (делаем объект вызываемым)
    void operator()(const HarmonicOscillator &state, HarmonicOscillator &dstate, double t) const {
        dstate[0] = state[1];  // dx/dt = v
        dstate[1] = -state[0]; // dv/dt = -x
    }

    // Метод для вывода состояния системы
    void observer(double t) const {
        std::cout << t << ": " << x << " " << v << std::endl;
    }

private:
    double x; // Положение
    double v; // Скорость
};

// Функция-обертка для вызова метода observer
void observer_wrapper(const HarmonicOscillator &state, double t) {
    state.observer(t);
}

int main() {
    // Начальные условия
    HarmonicOscillator state(1.0, 0.0); // x(0) = 1.0, v(0) = 0.0

    // Интегратор с использованием метода Рунге-Кутты 4-го порядка
    runge_kutta4<HarmonicOscillator> stepper;

    // Интегрируем систему ОДУ от t=0 до t=10 с шагом 0.1
    // Передаем сам объект HarmonicOscillator как вызываемый объект
    integrate_const(stepper, state, state, 0.0, 10.0, 0.1, observer_wrapper);

    return 0;
}
