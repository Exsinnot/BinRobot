import React from 'react'
import Bin from './Img/Bin.png'
import home from './Img/home.png'
import controller from './Img/game-controller.png'
import setting from './Img/settings.png'
import {Link} from 'react-router-dom'



function Sidebar() {
  return (
    <>
        {/* กรอบใหญ่สุด */}
        <div className="w-72 h-full bg-green-200">
            {/* กรอบของโลโก้กับชื่อ */}
            <div className='flex pt-4 shadow-xl rounded-lg'> 
                <div className="ml-2 logo w-24 flex-grow mb-4">
                    <img src={Bin} alt="My Image" />
                </div>
                <div className="flex justify-center items-center text-6xl namelogo w-32 flex-grow">
                    <h2 className='font-bold text-emerald-900'>BIN</h2>
                </div>
            </div> 
            {/* กรอบของพวก sidebar */}
            <nav className='mt-2'>
                <p className='text-3xl text-slate-400 mt-4'>menu</p>
                <ul>
                    <Link to='/Dashboard' className="flex items-center mt-2 py-4 shadow-md rounded-lg hover:bg-green-500 transition duration-300">
                        <img src={home} alt="Dashboard Icon" className="w-10 h-10 mr-2 ml-2"/>
                        <Link to='/Dashboard' className='text-3xl text-emerald-700 hover:text-white'>Dashboard</Link>
                    </Link>
                    <Link to='/Control' className="flex items-center mt-4 py-4 shadow-md rounded-lg hover:bg-green-500 transition duration-300">
                        <img src={controller} alt="Control Icon" className="w-10 h-10 mr-2 ml-2 " />
                        <Link to='/Control' className='text-3xl text-emerald-700 hover:text-white'>Control</Link>
                    </Link>
                    <Link to='/Setting' className="flex items-center mt-4 py-4 shadow-md rounded-lg hover:bg-green-500 transition duration-300">
                        <img src={setting} alt="Setting Icon" className="w-10 h-10 mr-2 ml-2" />
                        <Link to='/Setting' className='text-3xl text-emerald-700 hover:text-white'>Setting</Link>
                    </Link>
                </ul>
            </nav>
        </div>
    </>
    
  )
}

export default Sidebar