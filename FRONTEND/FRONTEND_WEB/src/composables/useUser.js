import { useRouter } from 'vue-router';
import axios from 'axios';

export function useUser() {
  const router = useRouter();

  function logout() {
    axios
      .post('https://robocop-backend-app.fly.dev/api/v1/auth/logout', null, {
        headers: {
          Authorization: `Bearer ${localStorage.getItem('accessToken')}`,
        },
      })
      .then(() => {
        // 클라이언트 측에서 토큰 제거
        localStorage.removeItem('accessToken');
        localStorage.removeItem('refreshToken');

        // Axios 기본 Authorization 헤더 제거
        delete axios.defaults.headers.common['Authorization'];

        // 사용자에게 알림 및 로그인 페이지로 이동
        alert('로그아웃 되었습니다.');
        router.push('/login');
      })
      .catch((error) => {
        console.error('로그아웃 실패:', error);
        alert('로그아웃에 실패했습니다.');
      });
  }

  return { logout };
}
