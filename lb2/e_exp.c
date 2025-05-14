/* Double-precision e^x function.
   Copyright (C) 2018-2025 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <https://www.gnu.org/licenses/>.  */

#include <math.h>
#include <stdint.h>
/* [ANNOTATION]
   Содержит макросы и инструменты для управления оптимизациями компилятора,
   которые необходимы для корректной работы математических функций. */
#include <math-barriers.h>
/* [ANNOTATION]
   Ещё вспомогательные функции для корректных вычислений. */
#include <math-narrow-eval.h>
/* [ANNOTATION]
   Обеспечивает совместимость математических функций со стандартом
   SVID (System V Interface Definition). Добавляет макросы для проверки
   необходимости включения совместимости с SVID. */
#include <math-svid-compat.h>
/* [ANNOTATION]
   Используется для создания псевдонимов математических функций, которые
   гарантированное работают только с конечными значениями (не NaN, inf и т.д).
   Это позволяет оптимизировать производительность и обеспечить корректное
   поведение функий в контекстах, где аргументы заведомо ограничены. */
#include <libm-alias-finite.h>
/* [ANNOTATION]
   Похожий на прошлый инструмент, создаёт alias'ы для double функций. */
#include <libm-alias-double.h>
/* [ANNOTATION]
   Содержит платформо-зависимые настройки и константы, используемые для
   оптимизации математических функиий (например, exp, sin, log). Служит
   централизованным местом для управления параметрами точности, алгоритмами
   и аппаратными особенностями. Позволяет адаптировать код под разные
   архитектуры и компиляторы. */
#include "math_config.h"

// [ANNOTATION] Размер таблицы предвычисленных значений
#define N (1 << EXP_TABLE_BITS)
// [ANNOTATION] Коэффициенты для редукции аргумента
#define InvLn2N __exp_data.invln2N
#define NegLn2hiN __exp_data.negln2hiN
#define NegLn2loN __exp_data.negln2loN
// [ANNOTATION] Константа для округления чисел (зависит от архитектуры)
#define Shift __exp_data.shift
// [ANNOTATION] Таблица предвычисленных значений для 2^{k/N}
#define T __exp_data.tab
// [ANNOTATION] Коэффициенты полинома, аппроксимирующего exp(r)
#define C2 __exp_data.poly[5 - EXP_POLY_ORDER]
#define C3 __exp_data.poly[6 - EXP_POLY_ORDER]
#define C4 __exp_data.poly[7 - EXP_POLY_ORDER]
#define C5 __exp_data.poly[8 - EXP_POLY_ORDER]

/* [ANNOTATION]
   Обрабатывает пограничные случаи:
   - Overflow: если результат превышает максимальное представимое значение
     double, возвращает HUGE_VAL с установкой флага переполнения.
   - Underflow: если результат близок к нулю, возвращает 0.0 с установкой
     флага потери точности.
/* Handle cases that may overflow or underflow when computing the result that
   is scale*(1+TMP) without intermediate rounding.  The bit representation of
   scale is in SBITS, however it has a computed exponent that may have
   overflown into the sign bit so that needs to be adjusted before using it as
   a double.  (int32_t)KI is the k used in the argument reduction and exponent
   adjustment of scale, positive k here means the result may overflow and
   negative k means the result may underflow.  */
static inline double
/* [ANNOTATION]
   tmp - значение аппроксимации r за вычетом 1.0;
   sbits - битовое представление числа scale (масштабирующего множителя
           2^{k/N} в формате IEEE754);
   ki - целое число k, полученное в результате редукции x. */
specialcase (double_t tmp, uint64_t sbits, uint64_t ki)
{
  double_t scale, y;

  if ((ki & 0x80000000) == 0) // k > 0
    {
      /* k > 0, the exponent of scale might have overflowed by <= 460.  */
      sbits -= 1009ull << 52;
      scale = asdouble (sbits);
      y = 0x1p1009 * (scale + scale * tmp);
      return check_oflow (y);
    }
  /* k < 0, need special care in the subnormal range.  */
  sbits += 1022ull << 52;
  scale = asdouble (sbits);
  y = scale + scale * tmp;
  if (y < 1.0)
    {
      /* Round y to the right precision before scaling it into the subnormal
	 range to avoid double rounding that can cause 0.5+E/2 ulp error where
	 E is the worst-case ulp error outside the subnormal range.  So this
	 is only useful if the goal is better than 1 ulp worst-case error.  */
      double_t hi, lo;
      lo = scale - y + scale * tmp;
      hi = 1.0 + y;
      lo = 1.0 - hi + y + lo;
      /* [ANNOTATION]
         Функция math_narrow_eval нужная для контроля над точностью
         промежуточных вычислений с плавающей точкой. Основной задачей
         является гарантирование того, что результаты выражений соответствуют
         ожидаемой точности типа данных (например, float или double), даже
         если компилятор или аппаратура используют более высокую точность
         (например, 80-битные регистры x87 на x86). */
      y = math_narrow_eval (hi + lo) - 1.0;
      /* Avoid -0.0 with downward rounding.  */
      if (WANT_ROUNDING && y == 0.0)
	y = 0.0;
      /* The underflow exception needs to be signaled explicitly.  */
      math_force_eval (math_opt_barrier (0x1p-1022) * 0x1p-1022);
    }
  y = 0x1p-1022 * y;
  return check_uflow (y);
}

// [ANNOTATION] Получение первых 12 бит x (знак и экспонента)
/* Top 12 bits of a double (sign and exponent bits).  */
static inline uint32_t
top12 (double x)
{
  return asuint64 (x) >> 52;
}

#ifndef SECTION
# define SECTION
#endif

double
/* [ANNOTATION]
   Раскрывается в инструкцию, которая размещает код функции в отдельной
   секции кода.
   Например для GCC: __attribute__((section(".text.exp"))). */
SECTION
__exp (double x)
{
  // [ANNOTATION] Экспонента числа x в формате IEEE754
  uint32_t abstop;
  /* [ANNOTATION]
     ki и idx -- индекс для доступа к таблице предвычисленных значений
     top и sbits */
  uint64_t ki, idx, top, sbits;
  /* double_t for better performance on targets with FLT_EVAL_METHOD==2.  */
  /* [ANNOTATION]
     kd, z, r -- промежуточные значения для вычислений
     r2, scale -- квадрат остатка r и мастабирующий множитель
     tail, tmp -- дополнительные члены для точного суммирования. */
  double_t kd, z, r, r2, scale, tail, tmp;

  abstop = top12 (x) & 0x7ff;
  /* [ANNOTATION]
     Разворачивается в __builtin_expect для GCC. Данный интринсик
     говорит компилятору, что первый аргумент (условие), скорее
     всего будет равен какому-то числу (0 или 1).
     
     Проверка, выходит ли экспонента x за пределы диапазона [2^-54, 512] */
  if (__glibc_unlikely (abstop - top12 (0x1p-54)
			>= top12 (512.0) - top12 (0x1p-54)))
    {
      /* [ANNOTATION]
         Обработка очень маленьких x. Если x настолько мал, что его
         экспонента меньше, чем 2^-54, то:
         - 1.0 + x (если требуется округление)
         - x (если округление не требуется)*/
      if (abstop - top12 (0x1p-54) >= 0x80000000)
	/* Avoid spurious underflow for tiny x.  */
	/* Note: 0 is common input.  */
	return WANT_ROUNDING ? 1.0 + x : 1.0;

      /* [ANNOTATION]
         Обработка underflow и overflow */
      if (abstop >= top12 (1024.0))
	{
	  if (asuint64 (x) == asuint64 (-INFINITY))
	    return 0.0;
	  if (abstop >= top12 (INFINITY))
	    return 1.0 + x;
	  if (asuint64 (x) >> 63)
	    return __math_uflow (0);
	  else
	    return __math_oflow (0);
	}
      /* [ANNOTATION]
         Если x находится в диапазоне [512, 1024), то код переходит
         к специальной обработке больших чисел, но не бесконечно
         больших. */
      /* Large x is special cased below.  */
      abstop = 0;
    }

  /* exp(x) = 2^(k/N) * exp(r), with exp(r) in [2^(-1/2N),2^(1/2N)].  */
  /* x = ln2/N*k + r, with int k and r in [-ln2/2N, ln2/2N].  */
  /* [ANNOTATION]
     Значение масштабированного аргумента. */
  z = InvLn2N * x;
  
/* [ANNOTATION]
   Используем интринсики, если они есть, иначе вычисляем
   вручную. Ниже вычисляется значение k для мастшабирования и
   дальнейших вычислений.
   kd -- мастиброванная штука, округлённая до целого числа
   ki -- ковертированный в integer тип kd */
#if TOINT_INTRINSICS
  kd = roundtoint (z);
  ki = converttoint (z);
#else
  /* z - kd is in [-1, 1] in non-nearest rounding modes.  */
  /* [ANNOTATION]
     См. строку 92. */
  kd = math_narrow_eval (z + Shift);
  ki = asuint64 (kd);
  kd -= Shift;
#endif
  /* [ANNOTATION]
     Вычисление остатка с использованием старшей и младшей части NegLn2N. */
  r = x + kd * NegLn2hiN + kd * NegLn2loN;
  /* 2^(k/N) ~= scale * (1 + tail).  */
  idx = 2 * (ki % N);
  top = ki << (52 - EXP_TABLE_BITS);
  tail = asdouble (T[idx]);
  /* This is only a valid scale when -1023*N < k < 1024*N.  */
  sbits = T[idx + 1] + top;
  /* exp(x) = 2^(k/N) * exp(r) ~= scale + scale * (tail + exp(r) - 1).  */
  /* Evaluation is optimized assuming superscalar pipelined execution.  */
  r2 = r * r;
  /* Without fma the worst case error is 0.25/N ulp larger.  */
  /* Worst case error is less than 0.5+1.11/N+(abs poly error * 2^53) ulp.  */
  tmp = tail + r + r2 * (C2 + r * C3) + r2 * r2 * (C4 + r * C5);
  if (__glibc_unlikely (abstop == 0))
    return specialcase (tmp, sbits, ki);
  scale = asdouble (sbits);
  /* Note: tmp == 0 or |tmp| > 2^-65 and scale > 2^-739, so there
     is no spurious underflow here even without fma.  */
  return scale + scale * tmp;
}
#ifndef __exp
hidden_def (__exp)
strong_alias (__exp, __ieee754_exp)
libm_alias_finite (__ieee754_exp, __exp)
# if LIBM_SVID_COMPAT
versioned_symbol (libm, __exp, exp, GLIBC_2_29);
libm_alias_double_other (__exp, exp)
# else
libm_alias_double (__exp, exp)
# endif
#endif
