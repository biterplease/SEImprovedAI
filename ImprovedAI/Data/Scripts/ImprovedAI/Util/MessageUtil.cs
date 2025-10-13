using ImprovedAI.Messages;

namespace ImprovedAI.Util
{
    public static class MessageUtil
    {
        public static string TopicToString(Channel topic)
        {
            switch (topic)
            {
                case Channel.DRONE_REGISTRATION:
                    return "DRONE_REGISTRATION";
                case Channel.DRONE_REPORTS:
                    return "DRONE_REPORTS";
                case Channel.DRONE_PERFORMANCE:
                    return "DRONE_PERFORMANCE";
                case Channel.DRONE_TASK_ASSIGNMENT:
                    return "DRONE_TASK_ASSIGNMENT";
                case Channel.LOGISTIC_REGISTRATION:
                    return "LOGISTIC_REGISTRATION";
                case Channel.LOGISTIC_UPDATE:
                    return "LOGISTIC_UPDATE";
                case Channel.LOGISTIC_REQUEST:
                    return "LOGISTIC_REQUEST";
                case Channel.LOGISTIC_PUSH:
                    return "LOGISTIC_PUSH";
                default:
                    return topic.ToString();
            }
        }
    }
}
