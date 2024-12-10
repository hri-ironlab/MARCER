using UnityEngine;
using RosMessageTypes.Std;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class PauseRuleMonitorClient : ROSBase
    {
        [SerializeField] private string _serviceName = "/rule_monitor/pause_rule_monitor";
        [SerializeField] private BoolVariable _isRuleMonitorPaused; 
        private void Start()
        {
            _ROS.RegisterRosService<TriggerRequest, TriggerResponse>(_serviceName);
        }
        public void RequestPauseRuleMonitor()
        {
            _isRuleMonitorPaused.Value = !_isRuleMonitorPaused.Value;
            _ROS.SendServiceMessage<TriggerResponse>(_serviceName,new TriggerResponse(), HandleResponse);
        }
        private void HandleResponse(TriggerResponse response)
        {
            return; 
        }

        private void OnApplicationQuit()
        {
            _isRuleMonitorPaused.Value = false;
        }
    }
}