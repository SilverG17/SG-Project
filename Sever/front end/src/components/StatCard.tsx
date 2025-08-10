// src/components/StatCard.tsx
type StatCardProps = {
  title: string;
  value: string;
};

const StatCard = ({ title, value }: StatCardProps) => (
  <div className="bg-white p-4 rounded shadow">
    <h3 className="text-sm text-gray-500">{title}</h3>
    <p className="text-xl font-bold">{value}</p>
  </div>
);

export default StatCard;