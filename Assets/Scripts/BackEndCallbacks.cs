using AOT;
using UnityEngine;

public static class BackEndCallbacks
{
    public delegate void StringCallback(string message);

    [MonoPInvokeCallback(typeof(StringCallback))]
    public static void DebugLog(string message)
    {
        Debug.Log("[c++] " + message);
    }

    [MonoPInvokeCallback(typeof(StringCallback))]
    public static void DebugLogWarning(string message)
    {
        Debug.LogWarning("[c++] " + message);
    }

    [MonoPInvokeCallback(typeof(StringCallback))]
    public static void DebugLogError(string message)
    {
        Debug.LogError("[c++] " + message);
    }
}
