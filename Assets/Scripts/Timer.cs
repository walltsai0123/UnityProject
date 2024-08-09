using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Timer
{
    public enum TimerOutputUnit
    {
        TIMER_OUTPUT_MICROSECONDS,
        TIMER_OUTPUT_MILLISECONDS,
        TIMER_OUTPUT_SECONDS,
        TIMER_OUTPUT_AUTO,
        TIMER_OUTPUT_TOTAL
    };
    private float start;
    private float end;
    private float elapse;
    private float pause_start;
    private float pause_end;
    private float pause_time;

    public void Tic()
    {
        start = Time.realtimeSinceStartup;
        pause_time = 0;
    }
    public void Toc()
    {
        end = Time.realtimeSinceStartup;
    }
    public void Pause()
    {
        pause_start = Time.realtimeSinceStartup;
    }
    public void Resume()
    {
        pause_end = Time.realtimeSinceStartup;
        pause_time += pause_end - pause_start;
    }
    // Duration in microseconds
    public float Duration()
    {
        elapse = end - start - pause_time;
        elapse *= 1000000f;
        return elapse;
    }

    // Duration in seconds
    public float DurationInSeconds()
    {
        elapse = end - start - pause_time;
        return elapse;
    }

    public void Report(string msg = "", TimerOutputUnit outputUnit = TimerOutputUnit.TIMER_OUTPUT_AUTO)
    {
        float d = Duration();
        switch (outputUnit)
        {
            case TimerOutputUnit.TIMER_OUTPUT_MICROSECONDS:
                Debug.LogFormat("{0}, time elapse: {1} us.\n", msg, d);
                break;
            case TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS:
                d = d * 0.001f;
                Debug.LogFormat("{0}, time elapse: {1} ms.\n", msg, d);
                break;
            case TimerOutputUnit.TIMER_OUTPUT_SECONDS:
                d = d * 0.000001f;
                Debug.LogFormat("{0}, time elapse: {1} s.\n", msg, d);
                break;
            case TimerOutputUnit.TIMER_OUTPUT_AUTO:
                // print in different scales
                if (d < 1000)
                {
                    Debug.LogFormat("{0}, time elapse: {1} us.\n", msg, d);
                }
                else if (d < 1000000)
                {
                    d = d * 0.001f;
                    Debug.LogFormat("{0}, time elapse: {1} ms.\n", msg, d);
                }
                else
                {
                    d = d * 0.000001f;
                    Debug.LogFormat("{0}, time elapse: {1} s.\n", msg, d);
                }
                break;
            default:
                break;
        }
    }
}
