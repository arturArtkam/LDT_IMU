#ifndef CXXLOG_H
#define CXXLOG_H

#include "textstream.h"
#include "if_constexpr.h"

enum Severity {LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR, LOG_FATAL};

namespace cxxlog {

enum Verbosity {QUIET, MINIMAL, NORMAL, VERBOSE}; /* тихий, минимальный, нормальный, многословный */

namespace detail {
static const char* severity_string[6] = {"trace", "debug", "info",
                                        "warn", "error", "fatal"};
} /* namespace detail */

template <typename... Attributes>
class Log
{
private:
    TextStream& _output_device; // Ссылка на устройство вывода
    bool _is_quiet_on_destruction; // Флаг для управления выводом в деструкторе

public:
    Log(const Log&) = delete;
    Log& operator =(const Log&) = delete;
    /* Конструктор принимает ссылку на проинициализированное (!) устройство вывода */
    Log(TextStream& output_dev, bool quiet_on_destruction = false) :
        _output_device(output_dev),
        _is_quiet_on_destruction(quiet_on_destruction)
    {
    }

    virtual ~Log()
    {
        if (!_is_quiet_on_destruction && verbosity() != Verbosity::QUIET) {
            _output_device << "\n Log destroyed.\n";
        }
    }

    TextStream& Get(Severity level = LOG_INFO)
    {
        switch (verbosity())
        {
            case Verbosity::QUIET:
                break;
            case Verbosity::MINIMAL:
                break;
            case Verbosity::NORMAL:
                OutputAttributes<Attributes...>(_output_device);
                break;
            case Verbosity::VERBOSE:
                OutputAttributes<Attributes...>(_output_device);
                _output_device << detail::severity_string[level] << " | ";
                break;
        }

        return _output_device;
    }

    static Verbosity& verbosity()
    {
        static Verbosity level = Verbosity::VERBOSE;
        return level;
    }

protected:
    /* Функция для одного атрибута */
    template <typename Attrib>
    TextStream& OutputAttributes(TextStream& os)
    {
        Attrib::Output(os);
        os << " | ";
        return os;
    }

    /*      Эта версия используется, если передано несколько типов Attrib1, Attrib2, ..., AttribN.
        Первый атрибут Attrib1 выводится в поток с помощью Attrib1::Output(os), затем в поток добавляется разделитель " | ".
        Функция вызывает сама себя с оставшимися типами Attribs..., тем самым обрабатывая их рекурсивно.
        Рекурсия продолжается, пока не останется ни одного атрибута. */
    template <typename Attrib1, typename... Attribs,
              typename = std::enable_if_t<(sizeof...(Attribs) > 0)>>
    TextStream& OutputAttributes(TextStream& os)
    {
        Attrib1::Output(os);
        os << " | ";
        return OutputAttributes<Attribs...>(os);
    }

    /* Базовый случай для рекурсии. Эта версия функции вызывается, если список атрибутов пуст (sizeof...(Attribs) == 0). */
    template <typename... Attribs,
              typename = std::enable_if_t<sizeof...(Attribs) == 0>>
    TextStream& OutputAttributes(TextStream& os)
    {
        return os;
    }
};

//template <typename... Attributes> // Убираем OStream, т.к. его роль не ясна
//class Log
//{
//private:
//    TextStream& _output_device; // Ссылка на устройство вывода
//    bool _is_quiet_on_destruction; // Флаг для управления выводом в деструкторе
//
//public:
//    Log(const Log&) = delete;
//    Log& operator =(const Log&) = delete;
//
//    // Конструктор принимает ссылку на устройство вывода
//    Log(TextStream& output_dev, bool quiet_on_destruction = false) :
//        _output_device(output_dev),
//        _is_quiet_on_destruction(quiet_on_destruction)
//    {
//        // Инициализация самого устройства вывода теперь ВНЕ класса Log
//    }
//
//    // Деструктор теперь более гибкий
//    // Если OStream::Output было для финального вывода данных (например, счетчика),
//    // это можно сделать через атрибут или специальный метод.
//    // Сейчас просто пример, что можно управлять этим.
//    ~Log() {
//        if (!_is_quiet_on_destruction && verbosity() != Verbosity::QUIET) {
//            // Если раньше OStream::Output(dev()) делало что-то важное,
//            // это нужно перенести. Например, если это был Counter,
//            // то Counter мог бы иметь метод flush() или print_summary(),
//            // который вызывался бы здесь или вручную пользователем.
//            // Для простоты пока уберем специфичный вызов.
//            // _output_device << "\nLog destroyed.\n"; // Пример
//        }
//    }
//
//    TextStream& Get(Severity level = LOG_INFO) {
//        // Проверка на QUIET лучше делать здесь, чтобы не вызывать OutputAttributes зря
//        if (verbosity() == Verbosity::QUIET) {
//            // Можно вернуть "пустой" поток-заглушку, если QUIET,
//            // но для простоты пока оставим так, полагаясь на то, что
//            // пользователь не будет писать в поток, если он QUIET.
//            // Или же TextStream должен иметь какой-то null_stream.
//        }
//
//        switch (verbosity()) {
//        case Verbosity::QUIET:
//            // Ничего не делаем или возвращаем null-stream
//            break;
//        case Verbosity::MINIMAL:
//            // Только сообщение
//            break;
//        case Verbosity::NORMAL:
//            OutputAttributes<Attributes...>(_output_device);
//            break;
//        case Verbosity::VERBOSE:
//            OutputAttributes<Attributes...>(_output_device);
//            _output_device << detail::severity_string[level] << " | ";
//            break;
//        }
//        return _output_device;
//    }
//
//    static Verbosity& verbosity() {
//        static Verbosity level = Verbosity::VERBOSE;
//        return level;
//    }
//
//    // dev() теперь не нужен в таком виде, т.к. устройство передается
//    // Вместо этого, функции print/print_log будут работать с конкретным экземпляром логгера
//
//protected:
//    template <typename Attrib>
//    TextStream& OutputAttributes(TextStream& os) {
//        Attrib::Output(os); // Предполагается, что атрибуты имеют статический метод Output
//        os << " | ";
//        return os;
//    }
//    /*      Эта версия используется, если передано несколько типов Attrib1, Attrib2, ..., AttribN.
//    Первый атрибут Attrib1 выводится в поток с помощью Attrib1::Output(os), затем в поток добавляется разделитель " | ".
//    Функция вызывает сама себя с оставшимися типами Attribs..., тем самым обрабатывая их рекурсивно.
//    Рекурсия продолжается, пока не останется ни одного атрибута. */
//    template <typename Attrib1, typename... Attribs,
//        typename = std::enable_if_t<(sizeof...(Attribs) > 0)>>
//        TextStream & OutputAttributes(TextStream& os) {
//        Attrib1::Output(os);
//        os << " | ";
//        return OutputAttributes<Attribs...>(os);
//    }
//
//    template <typename... Attribs,
//        typename = std::enable_if_t<sizeof...(Attribs) == 0>>
//        TextStream & OutputAttributes(TextStream& os) {
//        return os; // Базовый случай
//    }
//};
//
//class Counter;
//class Mode;
//
//typedef Log<Counter, Mode, Counter> Ak_log;
//extern Ak_log ak_log;

} /* namespace cxxlog */

__attribute__((always_inline)) inline void print(TextStream& stream) { stream << "\n\r"; }

template <class First, class... Other>
__attribute__((always_inline)) inline void print(TextStream& stream, const First &first, const Other&... other)
{
    stream << first << "";
    print(stream, other...);
}

template <Severity lvl, typename Logger_type, class... Args>
__attribute__((always_inline)) inline constexpr auto print_log(Logger_type& logger, const Args&... args)
{
    return
        ic::if_<(lvl >= LOG_INFO)>([&] { print(logger.Get(lvl), args...); },
        ic::else_([] { __NOP(); __NOP(); }));
}

#endif /* CXXLOG_H */
