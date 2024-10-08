//จะทำให้เหมาะกับโทรศัพท์กับไอแพดนะจ้ะ
import React from 'react';
import Bin from './Img/Bin.png';
import home from './Img/home.png';
import controller from './Img/game-controller.png';
import setting from './Img/settings.png';
import { Link } from 'react-router-dom';

function Navbar() {
  return (
    <div className="w-full bg-green-200 p-4 flex justify-between items-center shadow-xl rounded-lg ">
      <div className="flex flex-col items-center mb-4 sm:mb-0 sm:flex-row">
        <div className="ml-2 w-10 h-10 sm:w-12 h-12 mr-4 ">
          <img src={Bin} alt="My Image" />
        </div>
        <div className="text-3xl font-bold text-emerald-900 mr-4">BIN</div>
      </div>
      <div className="flex w-1 h-12 bg-green-400 mx-2 sm:hidden"></div> {/* Divider for large screens */}
      <nav className="flex space-x-4 w-full ">
        <Link to='/Dashboard' className="flex flex-col items-center shadow-md rounded-lg hover:bg-green-500 transition duration-300 p-2 rounded-lg w-20 sm:w-24">
          <img src={home} alt="Dashboard Icon" className="w-8 h-8" />
          <span className='text-emerald-700 text-sm'>Dashboard</span>
        </Link>
        <Link to='/Control' className="flex flex-col items-center shadow-md rounded-lg hover:bg-green-500 transition duration-300 p-2 rounded-lg w-20 sm:w-24">
          <img src={controller} alt="Control Icon" className="w-8 h-8" />
          <span className='text-emerald-700 text-sm'>Control</span>
        </Link>
        <Link to='/Setting' className="flex flex-col items-center shadow-md rounded-lg hover:bg-green-500 transition duration-300 p-2 rounded-lg w-20 sm:w-24">
          <img src={setting} alt="Setting Icon" className="w-8 h-8" />
          <span className='text-emerald-700 text-sm'>Setting</span>
        </Link>
      </nav>
    </div>
  );
}

export default Navbar;
