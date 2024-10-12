import React, { useState } from 'react';
import batteryfull from './Img/full.png';

const Imageswitcher = () => {
  // สร้าง state สำหรับเก็บค่าตัวเลข
  const [number, setNumber] = useState(0);

  // ฟังก์ชันสำหรับการเปลี่ยนตัวเลข
  const handleChange = (e) => {
    setNumber(parseInt(e.target.value));
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen">
      <input
        type="number"
        value={number}
        onChange={handleChange}
        className="border border-gray-300 p-2 mb-4"
        placeholder="Enter a number"
      />

      {/* ใช้เงื่อนไขในการแสดงรูปตามค่าตัวเลข */}
      {number >= 1 && number <= 50 ? (
        <img
          src={batteryfull}
          alt="Image 1"
          className="w-32 h-32"
        />
      ) : number > 50 && number <= 100 ? (
        <img
          src=""
          alt=""
          className="w-32 h-32"
        />
      ) : (
        <p>No image to display</p>
      )}
    </div>
  );
};

export default Imageswitcher;
