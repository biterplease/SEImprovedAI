using ImprovedAI.Messages;

namespace ImprovedAI.Utils
{
    public static class MessageUtil
    {
        public static string TopicToString(MessageTopics topic)
        {
            switch (topic)
            {
                case MessageTopics.DRONE_REGISTRATION:
                    return "DRONE_REGISTRATION";
                case MessageTopics.DRONE_REPORTS:
                    return "DRONE_REPORTS";
                case MessageTopics.DRONE_PERFORMANCE:
                    return "DRONE_PERFORMANCE";
                case MessageTopics.DRONE_TASK_ASSIGNMENT:
                    return "DRONE_TASK_ASSIGNMENT";
                case MessageTopics.LOGISTIC_REGISTRATION:
                    return "LOGISTIC_REGISTRATION";
                case MessageTopics.LOGISTIC_UPDATE:
                    return "LOGISTIC_UPDATE";
                case MessageTopics.LOGISTIC_REQUEST:
                    return "LOGISTIC_REQUEST";
                case MessageTopics.LOGISTIC_PUSH:
                    return "LOGISTIC_PUSH";
                default:
                    return topic.ToString();
            }
        }
    }
}
