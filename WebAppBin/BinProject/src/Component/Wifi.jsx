import React from 'react'
import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import WifiIcon from './Img/Bin.png'

function Wifi() {
  const [wifiName, setWifiName] = useState('');
  const [wifiPassword, setWifiPassword] = useState('');
  const [error, setError] = useState('');
  const navigate = useNavigate(); // ใช้ useNavigate สำหรับการนำทาง

  const handleConnect = (e) => {
    e.preventDefault();

    const validWifiName = "bin"; // Replace with the correct Wi-Fi name
    const validWifiPassword = "123456789"; // Replace with the correct Wi-Fi password

    if (wifiName === validWifiName && wifiPassword === validWifiPassword) {
      navigate('/Dashboard'); // เปลี่ยนเส้นทางไปยังหน้า Dashboard
    } else {
      setError('Invalid Wi-Fi Name or Password');
    }
  };
  return (
    <>
      <div className="flex items-center justify-center min-h-screen bg-green-200">
        <div className="bg-emerald-400 p-8 rounded-lg shadow-md max-w-md w-full text-center">
          <img className="mx-auto mb-4" src={WifiIcon} alt="Wi-Fi Icon" width="72" height="72" />
          <h1 className="text-emerald-900 text-2xl font-bold mb-6">Setting Wi-Fi</h1>
          <form className="space-y-4" onSubmit={handleConnect}>
            {error && <div className="text-red-500 text-xl font-bold mb-4">{error}</div>}
            <div>
              <label htmlFor="wifiName" className="block text-left text-sm font-medium text-gray-700 mb-2">SSID/Wi-Fi Name</label>
              <input type="text" 
                id="wifiName" 
                className="form-input mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm placeholder-gray-400 focus:outline-none focus:ring-blue-500 focus:border-emerald-900" 
                placeholder="Wi-Fi Name"
                value={wifiName}
                onChange={(e) => setWifiName(e.target.value)} // เมื่อผู้ใช้กรอกข้อมูลใหม่, สถานะจะถูกอัปเดต
              />
            </div>
            <div>
              <label htmlFor="wifiPassword" className="block text-left text-sm font-medium text-gray-700 mb-2">Wi-Fi Password</label>
              <input type="password" 
              id="wifiPassword" 
              className="form-input mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm placeholder-gray-400 focus:outline-none focus:ring-blue-500 focus:border-emerald-900" 
              placeholder="Wi-Fi Password" 
              value={wifiPassword}
              onChange={(e) => setWifiPassword(e.target.value)} // เมื่อผู้ใช้กรอกข้อมูลใหม่, สถานะจะถูกอัปเดต
              />
            </div>
            <button type="submit" className="w-full py-2 px-4 border border-transparent text-sm font-medium rounded-md text-white bg-emerald-900 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500">
            Connect
          </button>
        </form>
      </div>
    </div>
    </>
  )
}

export default Wifi