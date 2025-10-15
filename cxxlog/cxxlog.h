#ifndef CXXLOG_H
#define CXXLOG_H

#include "textstream.h"
#include "if_constexpr.h"

enum Severity {LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR, LOG_FATAL};

namespace cxxlog {

enum Verbosity {QUIET, MINIMAL, NORMAL, VERBOSE}; /* �����, �����������, ����������, ������������ */

namespace detail {
static const char* severity_string[6] = {"trace", "debug", "info",
                                        "warn", "error", "fatal"};
} /* namespace detail */

template <typename... Attributes>
class Log
{
private:
    TextStream& _output_device; // ������ �� ���������� ������
    bool _is_quiet_on_destruction; // ���� ��� ���������� ������� � �����������

public:
    Log(const Log&) = delete;
    Log& operator =(const Log&) = delete;
    /* ����������� ��������� ������ �� ��������������������� (!) ���������� ������ */
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
    /* ������� ��� ������ �������� */
    template <typename Attrib>
    TextStream& OutputAttributes(TextStream& os)
    {
        Attrib::Output(os);
        os << " | ";
        return os;
    }

    /*      ��� ������ ������������, ���� �������� ��������� ����� Attrib1, Attrib2, ..., AttribN.
        ������ ������� Attrib1 ��������� � ����� � ������� Attrib1::Output(os), ����� � ����� ����������� ����������� " | ".
        ������� �������� ���� ���� � ����������� ������ Attribs..., ��� ����� ����������� �� ����������.
        �������� ������������, ���� �� ��������� �� ������ ��������. */
    template <typename Attrib1, typename... Attribs,
              typename = std::enable_if_t<(sizeof...(Attribs) > 0)>>
    TextStream& OutputAttributes(TextStream& os)
    {
        Attrib1::Output(os);
        os << " | ";
        return OutputAttributes<Attribs...>(os);
    }

    /* ������� ������ ��� ��������. ��� ������ ������� ����������, ���� ������ ��������� ���� (sizeof...(Attribs) == 0). */
    template <typename... Attribs,
              typename = std::enable_if_t<sizeof...(Attribs) == 0>>
    TextStream& OutputAttributes(TextStream& os)
    {
        return os;
    }
};

//template <typename... Attributes> // ������� OStream, �.�. ��� ���� �� ����
//class Log
//{
//private:
//    TextStream& _output_device; // ������ �� ���������� ������
//    bool _is_quiet_on_destruction; // ���� ��� ���������� ������� � �����������
//
//public:
//    Log(const Log&) = delete;
//    Log& operator =(const Log&) = delete;
//
//    // ����������� ��������� ������ �� ���������� ������
//    Log(TextStream& output_dev, bool quiet_on_destruction = false) :
//        _output_device(output_dev),
//        _is_quiet_on_destruction(quiet_on_destruction)
//    {
//        // ������������� ������ ���������� ������ ������ ��� ������ Log
//    }
//
//    // ���������� ������ ����� ������
//    // ���� OStream::Output ���� ��� ���������� ������ ������ (��������, ��������),
//    // ��� ����� ������� ����� ������� ��� ����������� �����.
//    // ������ ������ ������, ��� ����� ��������� ����.
//    ~Log() {
//        if (!_is_quiet_on_destruction && verbosity() != Verbosity::QUIET) {
//            // ���� ������ OStream::Output(dev()) ������ ���-�� ������,
//            // ��� ����� ���������. ��������, ���� ��� ��� Counter,
//            // �� Counter ��� �� ����� ����� flush() ��� print_summary(),
//            // ������� ��������� �� ����� ��� ������� �������������.
//            // ��� �������� ���� ������ ����������� �����.
//            // _output_device << "\nLog destroyed.\n"; // ������
//        }
//    }
//
//    TextStream& Get(Severity level = LOG_INFO) {
//        // �������� �� QUIET ����� ������ �����, ����� �� �������� OutputAttributes ���
//        if (verbosity() == Verbosity::QUIET) {
//            // ����� ������� "������" �����-��������, ���� QUIET,
//            // �� ��� �������� ���� ������� ���, ��������� �� ��, ���
//            // ������������ �� ����� ������ � �����, ���� �� QUIET.
//            // ��� �� TextStream ������ ����� �����-�� null_stream.
//        }
//
//        switch (verbosity()) {
//        case Verbosity::QUIET:
//            // ������ �� ������ ��� ���������� null-stream
//            break;
//        case Verbosity::MINIMAL:
//            // ������ ���������
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
//    // dev() ������ �� ����� � ����� ����, �.�. ���������� ����������
//    // ������ �����, ������� print/print_log ����� �������� � ���������� ����������� �������
//
//protected:
//    template <typename Attrib>
//    TextStream& OutputAttributes(TextStream& os) {
//        Attrib::Output(os); // ��������������, ��� �������� ����� ����������� ����� Output
//        os << " | ";
//        return os;
//    }
//    /*      ��� ������ ������������, ���� �������� ��������� ����� Attrib1, Attrib2, ..., AttribN.
//    ������ ������� Attrib1 ��������� � ����� � ������� Attrib1::Output(os), ����� � ����� ����������� ����������� " | ".
//    ������� �������� ���� ���� � ����������� ������ Attribs..., ��� ����� ����������� �� ����������.
//    �������� ������������, ���� �� ��������� �� ������ ��������. */
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
//        return os; // ������� ������
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
