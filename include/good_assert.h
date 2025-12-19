#ifndef GOOD_ASSERT_H
#define GOOD_ASSERT_H

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

static inline void Assert(bool expr, const char *expr_str, const char *file, int line, const char *msg, ...)
{
    if (!expr)
    {
        printf("%s:%d: %s failed. ", file, line, expr_str);

        va_list args;
        va_start(args, msg);
        vprintf(msg, args);
        va_end(args);
        putc('\n', stdout);

        exit(1);
    }
}

#define ASSERT_EX(e, e_str, msg, ...) Assert(e, e_str, __FILE_NAME__, __LINE__, msg, __VA_ARGS__)
#define ASSERT(e, msg, ...) ASSERT_EX(e, #e, __VA_ARGS__)
#define ASSERT_GE_ZERO(a) ASSERT_EX(iszero(a) || (a) > 0, "(" #a ") >= 0", "Actual value: %.15lf", a)
#define ASSERT_ZERO(a) ASSERT_EX(iszero(a), "(" #a ") == 0", "Actual value: %.15lf", a)

#endif