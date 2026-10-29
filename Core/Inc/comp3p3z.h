/**
 * @file    comp3p3z.h
 * @brief   Latency-optimised 3-pole / 3-zero (3p3z) IIR compensator, float.
 *
 *   u[n] = B0*e[n] + B1*e[n-1] + B2*e[n-2] + B3*e[n-3]
 *                  + A1*u[n-1] + A2*u[n-2] + A3*u[n-3]
 *
 * The work is split in two halves so that the time from "ADC data available"
 * to "new duty written" is only one multiply-add:
 *
 *   u = comp3p3z_calc(&c, e);    // B0*e + precomputed sum, clamped
 *   ... write u (or the value actually applied) to the PWM ...
 *   comp3p3z_commit(&c, u_applied); // shift history, precompute next sum
 *
 * Passing the value that was REALLY applied to commit() is the anti-windup:
 * if the output was clamped, or the other loop (CV/CC) took over, the history
 * tracks the applied duty instead of winding up.
 */
#ifndef COMP3P3Z_H
#define COMP3P3Z_H

/* always_inline also works in -O0 Debug builds, where plain 'inline' is ignored */
#define COMP3P3Z_INLINE static inline __attribute__((always_inline))

typedef struct
{
    float b0, b1, b2, b3;   /* numerator (error) coefficients            */
    float a1, a2, a3;       /* denominator (output feedback) coefficients */
    float e0, e1, e2, e3;   /* error history (e0 = current sample)       */
    float u1, u2, u3;       /* applied-output history                    */
    float acc;              /* precomputed part of the next output       */
    float umin, umax;       /* output clamp                              */
} comp3p3z_t;

COMP3P3Z_INLINE void comp3p3z_reset(comp3p3z_t *c, float u0)
{
    c->e0 = c->e1 = c->e2 = c->e3 = 0.0f;
    c->u1 = c->u2 = c->u3 = u0;
    c->acc = (c->a1 + c->a2 + c->a3) * u0;
}

COMP3P3Z_INLINE void comp3p3z_init(comp3p3z_t *c,
                                 float b0, float b1, float b2, float b3,
                                 float a1, float a2, float a3,
                                 float umin, float umax)
{
    c->b0 = b0; c->b1 = b1; c->b2 = b2; c->b3 = b3;
    c->a1 = a1; c->a2 = a2; c->a3 = a3;
    c->umin = umin; c->umax = umax;
    comp3p3z_reset(c, umin);
}

/** Fast part: returns the clamped controller output for error e. */
COMP3P3Z_INLINE float comp3p3z_calc(comp3p3z_t *c, float e)
{
    c->e0 = e;
    float u = c->b0 * e + c->acc;
    if (!(u >= c->umin)) { u = c->umin; }   /* written this way it also maps NaN to umin */
    else if (u > c->umax) { u = c->umax; }
    return u;
}

/** Slow part: call after the PWM update with the value actually applied. */
COMP3P3Z_INLINE void comp3p3z_commit(comp3p3z_t *c, float u_applied)
{
    c->e3 = c->e2; c->e2 = c->e1; c->e1 = c->e0;
    c->u3 = c->u2; c->u2 = c->u1; c->u1 = u_applied;
    c->acc = c->b1 * c->e1 + c->b2 * c->e2 + c->b3 * c->e3
           + c->a1 * c->u1 + c->a2 * c->u2 + c->a3 * c->u3;
}

#endif /* COMP3P3Z_H */
