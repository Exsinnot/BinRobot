import React, { useState, useEffect } from 'react';
import bin from './Img/trash.png';
import Sidebar from './Sidebar';
import binvvv1 from './Img/22.png';
import binvvv2 from './Img/23.png';
import binvvv3 from './Img/24.png';
import binvvv4 from './Img/25.png';
import binvvv5 from './Img/26.png';
import binvvv6 from './Img/27.png';
import binvvv7 from './Img/28.png';
import batteryfull from './Img/full.png';
import empty from './Img/empty.png';
import verylow from './Img/low3.png';
import low from './Img/low.png';
import middle from './Img/midiem.png';
import Navbar from './Navbar';
import Grapt from './Grapt';
import TrashAlert from './Trashalert'; // นำเข้าคอมโพเนนต์ TrashAlert

function Dashboard() {
  const [trashLevel, setTrashLevel] = useState(0); // ระดับขยะเริ่มต้น
  const [alertVisible, setAlertVisible] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [SSIDBin, setSSIDBin] = useState("EXINNOT");
  const [StatusBin, setStatusBin] = useState("โหมดที่ 1 : เดินสุ่ม");
  
  useEffect(() => {
    const interval = setInterval(() => {
      setTrashLevel((prevTrashLevel) => {
        if (prevTrashLevel < 100) {
          return prevTrashLevel + 10;
        } else {
          return 0; // รีเซ็ตกลับไปที่ 0 เมื่อถึง 100
        }
      });
    }, 1000); // เพิ่มขึ้นทุกๆ 1 วินาที

    // ล้างการทำงานของ interval เมื่อคอมโพเนนต์ถูกยกเลิก
    return () => clearInterval(interval);
  }, []);
  useEffect(() => {
    const interval = setInterval(() => {
      setBatteryLevel((prevTrashLevel) => {
        if (prevTrashLevel < 100) {
          return prevTrashLevel + 10;
        } else {
          return 0; // รีเซ็ตกลับไปที่ 0 เมื่อถึง 100
        }
      });
    }, 1000); // เพิ่มขึ้นทุกๆ 1 วินาที

    // ล้างการทำงานของ interval เมื่อคอมโพเนนต์ถูกยกเลิก
    return () => clearInterval(interval);
  }, []);

  // const fetchData = () => {
  //   fetch('http://192.168.137.232:8000/getdatadb')
  //     .then(response => {
  //       if (!response.ok) {
  //         throw new Error('Network response was not ok.');
  //       }
  //       return response.json();
  //     })
  //     .then(data => {
  //       console.log(data);
  //       setBatteryLevel(parseInt(data.Battery));
  //       setTrashLevel(data.Trash);
  //       setSSIDBin(data.SSID)
  //       setStatusBin(data.Status)
  //     })
  //     .catch(error => {
  //       console.error('Error', error);
  //     });
  // };

  // useEffect(()=>{
  //   fetchData();
  //   const interval = setInterval(fetchData,20000);
    
  //   return ()=> clearInterval(interval);
  // },[]);

  // useEffect(() => {
  //   // ตรวจสอบระดับขยะและแสดงการแจ้งเตือนหากถึง 100%
  //   if (trashLevel >= 100) {
  //     setAlertVisible(true);
  //   }
  // }, [trashLevel]);

  // const closeAlert = () => {
  //   setAlertVisible(false);
  // };

  return (
    <>
    {alertVisible && <TrashAlert onClose={closeAlert} />}
    <div className="relative flex flex-col lg:flex-row h-screen">
      <div className='fixed hidden lg:block lg:w-1/5 h-screen top-0 left-0 z-50'>
        <Sidebar />
      </div>

      <div className='flex flex-col flex-grow w-full lg:ml-72'>
        <div className='top-0 w-full block lg:hidden'>
          <Navbar />
        </div>

        <div className="flex-grow grid grid-cols-3 grid-rows-[auto,auto,auto,1fr] gap-4 p-4 lg:p-8 
                        portrait-mobile:p-4 landscape-mobile:grid-rows-[auto,auto,auto,1fr] landscape-mobile:h-screen landscape-mobile:p-4 
                        portrait-desktop:grid-rows-[auto,auto,auto,1fr] portrait-desktop:grid-cols-3">

          {/* กล่องที่ 1 */}
          <div className="col-span-3 row-span-1 bg-green-200 shadow-md rounded-lg border-2 border-green-300 flex flex-col justify-start items-start font-bold p-4 md:col-span-2 row-span-2 
                          landscape-mobile:row-span-3 portrait-desktop:col-span-3 portrait-desktop:row-span-1">
            <h1 className='portrait-mobile:text-xl landscape-mobile:text-xl md:text-4xl'>แบตเตอรี่</h1>
            <div className='flex flex-col justify-start items-center w-full h-full'>
            {batteryLevel ===0 ? (
                <img src={empty} 
                alt="" 
                className='w-96 landscape-mobile:w-28 portrait-desktop:w-40'/>
              ):batteryLevel >=1 && batteryLevel <=25 ? (
                <img src={verylow} 
                alt="" 
                className='w-96 landscape-mobile:w-28 portrait-desktop:w-40'/>
              ):batteryLevel >=26 && batteryLevel <=50 ? (
                <img src={low} 
                alt="" 
                className='w-96 landscape-mobile:w-28 portrait-desktop:w-40'/>
              ):batteryLevel >=51 && batteryLevel <=75 ? (
                <img src={middle} 
                alt="" 
                className='w-96 landscape-mobile:w-28 portrait-desktop:w-40'/>
              ):batteryLevel >=76 && batteryLevel <=100 ? (
                <img src={batteryfull} 
                alt="" 
                className='w-96 landscape-mobile:w-28 portrait-desktop:w-40'/>
              ): null
              }
              <div className='flex w-full justify-around mt-2 md:w-full portrait-desktop:mt-0'>
                <div className='flex flex-col items-center mb-8'>
                  <span className='text-sm md:text-lg landscape-mobile:text-sm'>แบตเตอรี่</span>
                  <span className='text-3xl md:text-5xl landscape-mobile:text-3xl portrait-desktop:text-4xl'>{batteryLevel}%</span>
                </div>
                <div className='flex flex-col items-center '>
                  <span className='text-sm md:text-lg landscape-mobile:text-sm'>อัตราการเสื่อสภาพ</span>
                  <span className='text-3xl md:text-5xl landscape-mobile:text-3xl portrait-desktop:text-4xl'>0%</span>
                </div>
              </div>
            </div>
          </div>

          {/* กล่องที่ 2 */}
          <div className="col-span-3 row-span-1 bg-lime-200 shadow-md rounded-lg border-2 border-lime-300 flex flex-col justify-start items-start text-2xl font-bold p-4 md:col-span-1 row-span-3 
                          landscape-mobile:row-span-4 portrait-desktop:col-span-3 portrait-desktop:row-span-1">
            <h1 className='portrait-mobile:text-xl md:text-4xl landscape-mobile:text-xl'>ถังขยะ</h1>
            <div className='flex flex-col justify-start items-center w-full h-full'>
              {trashLevel ===0 ? (
                  <img src={binvvv1} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=1 && trashLevel <=16 ? (
                  <img src={binvvv2} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=17 && trashLevel <=32 ? (
                  <img src={binvvv3} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=33 && trashLevel <=48 ? (
                  <img src={binvvv4} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=49 && trashLevel <=64 ? (
                  <img src={binvvv5} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=65 && trashLevel <=80 ? (
                  <img src={binvvv6} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ):trashLevel >=81 && trashLevel <=100? (
                  <img src={binvvv7} 
                  alt="" 
                  className='w-36 mt-4 md:w-96 md:mt-24 landscape-mobile:w-36 landscape-mobile:mt-8 portrait-desktop:w-36 portrait-desktop:mt-2'/>
                ): null
              }
              <div className='flex flex-col items-center mt-4 md:mt-16 landscape-mobile:mt-8 portrait-desktop:mt-4'>
                <span className='text-sm md:text-lg landscape-mobile:text-sm'>ปริมาณขยะ</span>
                <span className='text-3xl md:text-5xl landscape-mobile:text-3xl portrait-desktop:text-4xl'>{trashLevel}%</span>
              </div>
            </div>
          </div>

          {/* กล่องที่ 3 */}
          <div className="col-span-3 row-span-1 bg-teal-200 shadow-md rounded-lg border-2 border-teal-300 flex flex-col justify-start items-start text-2xl font-bold p-4 md:col-span-1 row-span-1 
                          portrait-desktop:col-span-3 portrait-desktop:row-span-1">
            <h1 className='text-2xl md:text-4xl portrait-mobile:text-xl landscape-mobile:text-xl'>สถานะ</h1>
            <div className='flex w-full justify-around'>
              <span className='text-xl text-gray-700 md:text-lg landscape-mobile:text-base'>{StatusBin}</span>
            </div>
          </div>

          {/* กล่องที่ 4 */}
          <div className="col-span-3 row-span-1 bg-cyan-200 shadow-md rounded-lg border-2 border-cyan-300 flex flex-col justify-start items-start text-2xl font-bold p-4 md:col-span-1 row-span-1 
                          portrait-desktop:col-span-3 portrait-desktop:row-span-1">
            <h1 className='text-2xl md:text-4xl portrait-mobile:text-xl landscape-mobile:text-xl'>WI-FI</h1>
            <div className='flex w-full justify-around'>
              <span className='text-xl text-gray-700 md:text-lg landscape-mobile:text-base'>{SSIDBin}</span>
            </div>
          </div>

          {/* กล่องที่ 5 (กราฟ) */}
          <div className="col-span-3 row-span-1 bg-sky-200 shadow-md rounded-lg border-2 border-sky-300 flex flex-col justify-start items-center p-4 lg:p-8">
            <div className='w-full h-full'>
              <Grapt/>
            </div>
          </div>
        </div>
      </div>
    </div>
    </>
    
  );
}

export default Dashboard;
