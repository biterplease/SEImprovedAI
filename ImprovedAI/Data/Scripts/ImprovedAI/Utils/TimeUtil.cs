using System;


namespace ImprovedAI.Utils
{
    public static class TimeUtil
    {
        public const double TICKS_PER_SECOND = 60.0;
        public const double MILLISECONDS_PER_TICK = 1000.0 / TICKS_PER_SECOND; // 16.666... ms
        public const double TICKS_PER_MILLISECOND = TICKS_PER_SECOND / 1000.0; // 0.06 ticks
        /// <summary>
        /// Convert ticks to MS, rounded down
        /// </summary>
        /// <param name="ticks"></param>
        /// <returns></returns>
        public static int TickToMs(int ticks)
        {
            return (int)(ticks * MILLISECONDS_PER_TICK);
        }
        /// <summary>
        /// Convert milliseconds to server ticks
        /// </summary>
        /// <param name="ms">Time in milliseconds</param>
        /// <returns>Equivalent number of server ticks (rounded)</returns>
        public static int MsToTick(int ms)
        {
            return (int)Math.Floor(ms * TICKS_PER_MILLISECOND);
        }
        /// <summary>
        /// Convert TimeSpan to server ticks
        /// </summary>
        /// <param name="timeSpan">TimeSpan to convert</param>
        /// <returns>Equivalent number of server ticks (rounded)</returns>
        public static int TimeSpanToTick(TimeSpan timeSpan)
        {
            return MsToTick((int)timeSpan.TotalMilliseconds);
        }

        /// <summary>
        /// Convert server ticks to TimeSpan
        /// </summary>
        /// <param name="ticks">Number of server ticks</param>
        /// <returns>Equivalent TimeSpan</returns>
        public static TimeSpan TickToTimeSpan(int ticks)
        {
            return TimeSpan.FromMilliseconds(TickToMs(ticks));
        }
    }
}