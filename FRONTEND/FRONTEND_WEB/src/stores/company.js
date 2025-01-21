import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useCompanyStore = defineStore('company', () => {
  const companyName = ref('')
  const adminName = ref('관리자')

  function setCompanyName(name) {
    companyName.value = name
  }

  function setAdminName(name) {
    adminName.value = name
  }

  return {
    companyName,
    adminName,
    setCompanyName,
    setAdminName
  }
}) 