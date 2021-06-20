using MLAPI;
using UnityEngine;
using UnityEngine.UI;

namespace Game.UI
{
    public class MenuBehaviour : MonoBehaviour
    {
        [SerializeField] private GameObject hostButtonObject;
        [SerializeField] private GameObject clientButtonObject;
        [SerializeField] private GameObject serverButtonObject;

        private Button _hostButton;
        private Button _clientButton;
        private Button _serverButton;

        private void Start()
        {
            _hostButton = hostButtonObject.GetComponent<Button>();
            _hostButton.onClick.AddListener(HandleHostButtonClick);
            
            _clientButton = clientButtonObject.GetComponent<Button>();
            _clientButton.onClick.AddListener(HandleClientButtonClick);
            
            _serverButton = serverButtonObject.GetComponent<Button>();
            _serverButton.onClick.AddListener(HandleServerButtonClick);
        }

        private void HandleHostButtonClick()
        {
            NetworkManager.Singleton.StartHost();
            HideMenu();
        }
        
        private void HandleClientButtonClick()
        {
            NetworkManager.Singleton.StartClient();
            HideMenu();
        }
        
        private void HandleServerButtonClick()
        {
            NetworkManager.Singleton.StartServer();
            HideMenu();
        }

        private void HideMenu()
        {
            gameObject.SetActive(false);
        }
    }
}